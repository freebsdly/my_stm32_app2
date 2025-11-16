use defmt::info;
use stm32f4xx_hal::{pac, rcc::Rcc};

const SDRAM_BASE_ADDR: u32 = 0xC000_0000;
const SDRAM_SIZE: u32 = 32 * 1024 * 1024;

pub struct SdramDriver {
    fmc: pac::FMC,
}

impl SdramDriver {
    pub fn new(fmc: pac::FMC) -> Self {
        Self { fmc }
    }

    pub fn debug_config(&self) {
        // 打印SDCR寄存器配置
        let sdcr = self.fmc.sdcr1().read().bits();
        info!("SDCR 配置: 0x{:08X}", sdcr);
        info!(
            "  地址位数: COL={}bit, ROW={}bit",
            (sdcr & 0b11) + 8,         // 列地址位数应该加8
            ((sdcr >> 2) & 0b11) + 11  // 行地址位数应该加11
        );
        info!(
            "  数据宽度: {}bit",
            match (sdcr >> 4) & 0b11 {
                0 => 8,
                1 => 16,
                2 => 32,
                _ => 0,
            }
        );
        info!(
            "  内部Bank数: {}",
            if (sdcr >> 6) & 0b1 == 0 { 2 } else { 4 }
        );
        let cas_value = (sdcr >> 7) & 0b11;
        let actual_cas = match cas_value {
            1 => 2, // 01 -> 2周期
            2 => 3, // 10 -> 3周期
            _ => 0, // 非法
        };
        info!("CAS延迟: {}周期", actual_cas);
        info!(
            "  写保护: {}",
            if (sdcr >> 9) & 0b1 == 0 {
                "禁用"
            } else {
                "启用"
            }
        );
        info!(
            "  SDClock周期: {}",
            match (sdcr >> 10) & 0b11 {
                0b00 => "Disabled",
                0b10 => "HCLK/2", // 二进制10对应十进制2
                0b11 => "HCLK/3", // 二进制11对应十进制3
                _ => "Reserved",
            }
        );
        info!(
            "  突发读取: {}",
            if (sdcr >> 12) & 0b1 == 0 {
                "禁用"
            } else {
                "启用"
            }
        );
        info!(
            "  读管道: {} HCLK",
            match (sdcr >> 13) & 0b11 {
                0 => 0,
                1 => 1,
                2 => 2,
                _ => 0,
            }
        );

        // 打印SDTR寄存器配置
        let sdtr = self.fmc.sdtr1().read().bits();
        info!("SDTR 配置: 0x{:08X}", sdtr);
        info!("  TMRD (模式寄存器设置时间): {} cycles", (sdtr & 0xF) + 1);
        info!(
            "  TXSR (退出自刷新时间): {} cycles",
            ((sdtr >> 4) & 0xF) + 1
        );
        info!("  TRAS (自刷新时间): {} cycles", ((sdtr >> 8) & 0xF) + 1);
        info!("  TRC (行循环延迟): {} cycles", ((sdtr >> 12) & 0xF) + 1);
        info!("  TWR (恢复延迟): {} cycles", ((sdtr >> 16) & 0xF) + 1);
        info!("  TRP (行预充电延迟): {} cycles", ((sdtr >> 20) & 0xF) + 1);
        info!("  TRCD (行到列延迟): {} cycles", ((sdtr >> 24) & 0xF) + 1);

        // 打印SDRTR寄存器配置
        let sdrtr = self.fmc.sdrtr().read().bits();
        info!("SDRTR 配置: 0x{:08X}", sdrtr);
        info!("  刷新计数器: {}", sdrtr & 0x1FFF);
        info!(
            "  清除刷新错误标志: {}",
            if (sdrtr >> 31) & 0b1 == 1 {
                "已清除"
            } else {
                "未清除"
            }
        );

        // 打印SDSR状态
        let sdsr = self.fmc.sdsr().read().bits();
        info!("SDSR 状态: 0x{:08X}", sdsr);
        info!("  刷新错误: {}", if sdsr & 0b1 == 1 { "是" } else { "否" });
        info!(
            "  Bank1模式: {}",
            match (sdsr >> 1) & 0b11 {
                0 => "正常",
                1 => "自刷新",
                2 => "预充电",
                3 => "加载模式寄存器",
                _ => "未知",
            }
        );
        info!(
            "  Bank2模式: {}",
            match (sdsr >> 3) & 0b11 {
                0 => "正常",
                1 => "自刷新",
                2 => "预充电",
                3 => "加载模式寄存器",
                _ => "未知",
            }
        );
        info!(
            "  忙状态: {}",
            if (sdsr >> 5) & 0b1 == 1 {
                "忙"
            } else {
                "空闲"
            }
        );
    }

    /// 向SDRAM发送命令
    pub fn send_cmd(
        &mut self,
        bankx: u8,
        cmd: u8,
        refresh: u8,
        regval: u16,
    ) -> Result<(), &'static str> {
        let mut tempreg: u32 = 0;
        let mut retry: u32 = 0;

        tempreg |= (cmd as u32) << 0; // 设置指令
        tempreg |= 1 << (4 - bankx);
        tempreg |= (refresh as u32) << 5; // 设置自刷新次数
        tempreg |= (regval as u32) << 9; // 设置模式寄存器的值

        // 配置寄存器
        unsafe { self.fmc.sdcmr().write(|w| w.bits(tempreg)) };

        // 等待指令发送完成
        while self.fmc.sdsr().read().bits() & (1 << 5) != 0 {
            retry += 1;
            if retry > 0x1FFFFF {
                return Err("SDRAM command timeout");
            }
        }

        Ok(())
    }

    /// 初始化SDRAM
    pub fn init(&mut self, rcc: &mut Rcc) -> Result<(), &'static str> {
        // 1. 使能时钟（修复：同时使能GPIO和FMC时钟）
        rcc.ahb3enr().modify(|_, w| w.fmcen().set_bit());
        rcc.ahb1enr().modify(|_, w| {
            w.gpiocen()
                .set_bit()
                .gpioden()
                .set_bit()
                .gpioeen()
                .set_bit()
                .gpiofen()
                .set_bit()
                .gpiogen()
                .set_bit()
        });
        cortex_m::asm::delay(100);
        info!("FMC时钟已使能");

        // 确保FMC初始化前SDRAM处于复位状态
        self.fmc.sdcr1().reset();
        self.fmc.sdcr2().reset();
        cortex_m::asm::delay(100);

        // 3. 控制寄存器配置 (同时配置SDCR1和SDCR2)
        let mut sdctrlreg: u32 = 0;
        sdctrlreg &= !(0b11 << 7); // 清除原CAS设置
        sdctrlreg |= 1 << 0; // 9位列地址
        sdctrlreg |= 2 << 2; // 13位行地址
        sdctrlreg |= 1 << 4; // 16位数据位宽
        sdctrlreg |= 1 << 6; // 4个内部存储区
        sdctrlreg |= 0b01 << 7; // 2个CAS延迟 (正确值应为0b10)
        sdctrlreg |= 0 << 9; // 允许写访问
        sdctrlreg |= 2 << 10; // SDRAM时钟=HCLK/2 (正确值应为0b10)
        sdctrlreg |= 1 << 12; // 使能突发访问
        sdctrlreg |= 0 << 13; // 读通道延迟0个HCLK

        // 多次写入确保配置生效
        for _ in 0..3 {
            self.fmc.sdcr1().write(|w| unsafe { w.bits(sdctrlreg) });
            self.fmc.sdcr2().write(|w| unsafe { w.bits(sdctrlreg) });
            cortex_m::asm::delay(100);
        }

        info!("控制寄存器配置完成: 0x{:08X}", sdctrlreg);

        // 2. 时序寄存器配置
        let mut sdtimereg: u32 = 0;
        sdtimereg |= 1 << 0; // 加载模式寄存器到激活时间延迟=2周期
        sdtimereg |= 6 << 4; // 退出自刷新延迟=7周期
        sdtimereg |= 5 << 8; // 自刷新时间=6周期
        sdtimereg |= 5 << 12; // 行循环延迟=6周期
        sdtimereg |= 1 << 16; // 恢复延迟=2周期
        sdtimereg |= 1 << 20; // 行预充电延迟=2周期
        sdtimereg |= 1 << 24; // 行到列延迟=2周期

        self.fmc.sdtr1().write(|w| unsafe { w.bits(sdtimereg) });
        // self.fmc.sdtr2().write(|w| unsafe { w.bits(sdtimereg) });
        cortex_m::asm::delay(100);

        info!("时序寄存器配置完成: 0x{:08X}", sdtimereg);

        // 3. SDRAM初始化序列
        info!("开始SDRAM初始化序列...");

        // 步骤1: 时钟配置使能
        self.send_cmd(0, 1, 0, 0)?;
        info!("步骤1: 时钟配置使能完成");

        // 延迟至少500us
        self.wait_busy(500);

        // 步骤2: 预充电所有存储区
        self.send_cmd(0, 2, 0, 0)?;
        self.wait_busy(100);
        info!("步骤2: 预充电完成");

        // 步骤3: 自动刷新（执行8次）
        self.send_cmd(0, 3, 8, 0)?;
        self.wait_busy(100);
        info!("步骤3: 自动刷新完成");

        // 4. 加载模式寄存器
        let mut mregval: u16 = 0;
        mregval |= 1 << 0; // 突发长度:1
        mregval |= 0 << 3; // 突发类型:连续
        mregval |= 2 << 4; // CAS值:2
        mregval |= 0 << 7; // 操作模式:标准模式
        mregval |= 1 << 9; // 突发写模式:单点访问

        self.send_cmd(0, 4, 0, mregval)?;
        self.wait_busy(100);
        info!("步骤4: 模式寄存器加载完成");

        // 5. 设置刷新定时器
        let refresh_rate = 730;

        // 多次写入确保配置生效
        for _ in 0..3 {
            self.fmc.sdrtr().write(|w| unsafe { w.bits(refresh_rate) });
            cortex_m::asm::delay(100);
        }

        info!("刷新定时器设置: {}", refresh_rate);

        // 6. 最后再等待一段时间让SDRAM稳定
        self.wait_busy(1000);

        info!("SDRAM初始化完成");
        Ok(())
    }

    /// 优化后的读取函数 - 支持多种数据宽度
    pub fn read<T: Copy + Sized>(&self, addr: u32) -> T {
        unsafe {
            let ptr = (SDRAM_BASE_ADDR + addr) as *const T;
            ptr.read_volatile()
        }
    }

    /// 批量读取函数 - 大幅提升效率
    pub fn read_slice<T: Copy + Sized>(&self, addr: u32, data: &mut [T]) {
        unsafe {
            let src_ptr = (SDRAM_BASE_ADDR + addr) as *const T;
            let src_slice = core::slice::from_raw_parts(src_ptr, data.len());
            data.copy_from_slice(src_slice);
        }
    }

    /// 专门优化的字节缓冲区读取
    pub fn read_buffer(&self, buffer: &mut [u8], addr: u32, n: usize) {
        // 使用批量读取提升效率
        self.read_slice::<u16>(addr, &mut unsafe {
            core::slice::from_raw_parts_mut(buffer.as_mut_ptr() as *mut u16, n / 2)
        });

        // 处理奇数长度情况
        if n % 2 == 1 {
            buffer[n - 1] = self.read::<u8>(addr + n as u32 - 1);
        }
    }

    pub fn write<T: Copy + Sized>(&self, addr: u32, value: T) {
        unsafe {
            let ptr = (SDRAM_BASE_ADDR + addr) as *mut T;
            ptr.write_volatile(value);
        }
    }

    pub fn write_slice<T: Copy + Sized>(&self, addr: u32, data: &[T]) {
        unsafe {
            let dst_ptr = (SDRAM_BASE_ADDR + addr) as *mut T;
            let dst_slice = core::slice::from_raw_parts_mut(dst_ptr, data.len());
            dst_slice.copy_from_slice(data);
        }
    }

    pub fn write_buffer(&self, buffer: &[u8], addr: u32) {
        // 批量写入主要数据
        self.write_slice::<u16>(addr, &unsafe {
            core::slice::from_raw_parts(buffer.as_ptr() as *const u16, buffer.len() / 2)
        });

        // 处理剩余字节
        if buffer.len() % 2 == 1 {
            let last_byte = buffer[buffer.len() - 1];
            self.write::<u8>(addr + buffer.len() as u32 - 1, last_byte);
        }
    }

    fn is_valid_address(&self, addr: u32, size: usize) -> bool {
        // 检查地址范围是否在SDRAM容量内 (32MB)
        (addr + size as u32) <= SDRAM_SIZE
    }

    pub fn wait_busy(&self, clocks: u32) {
        let ticks = clocks * (180_000_000 / 1_000_000); // 180MHz系统时钟
        cortex_m::asm::delay(ticks);
    }
}
