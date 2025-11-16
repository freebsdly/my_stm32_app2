use defmt::info;
use stm32f4xx_hal::{pac, rcc::Rcc};

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
        info!("  CAS延迟: {}", ((sdcr >> 7) & 0b11) + 1); // 直接加1显示实际延迟
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
                0 => "Disable",
                1 => "HCLK/2",
                2 => "HCLK/3",
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
        sdctrlreg |= 1 << 0; // 9位列地址
        sdctrlreg |= 2 << 2; // 13位行地址
        sdctrlreg |= 1 << 4; // 16位数据位宽
        sdctrlreg |= 1 << 6; // 4个内部存储区
        sdctrlreg |= 2 << 7; // 2个CAS延迟 (正确值应为0b10)
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
        sdtimereg |= 1 << 24; // TRCD=2周期 ✓ (1+1=2)

        // 多次写入确保配置生效
        for _ in 0..3 {
            self.fmc.sdtr1().write(|w| unsafe { w.bits(sdtimereg) });
            self.fmc.sdtr2().write(|w| unsafe { w.bits(sdtimereg) });
            cortex_m::asm::delay(100);
        }

        info!("时序寄存器配置完成: 0x{:08X}", sdtimereg);

        // 3. SDRAM初始化序列
        info!("开始SDRAM初始化序列...");

        // 步骤1: 时钟配置使能
        self.send_cmd(0, 1, 0, 0)?;
        info!("步骤1: 时钟配置使能完成");

        // 延迟至少500us
        cortex_m::asm::delay(500 * 180); // 192MHz系统时钟

        // 步骤2: 预充电所有存储区
        self.send_cmd(0, 2, 0, 0)?;
        cortex_m::asm::delay(100 * 180); // 100us延迟
        info!("步骤2: 预充电完成");

        // 步骤3: 自动刷新（执行8次）
        self.send_cmd(0, 3, 8, 0)?;
        cortex_m::asm::delay(100 * 180); // 增加延时
        info!("步骤3: 自动刷新完成");

        // 4. 加载模式寄存器
        let mut mregval: u16 = 0;
        mregval |= 1 << 0; // 突发长度:1
        mregval |= 0 << 3; // 突发类型:连续
        mregval |= 2 << 4; // CAS值:2
        mregval |= 0 << 7; // 操作模式:标准模式
        mregval |= 1 << 9; // 突发写模式:单点访问

        self.send_cmd(0, 4, 0, mregval)?;
        cortex_m::asm::delay(100 * 180); // 100us延迟
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
        cortex_m::asm::delay(1000 * 180); // 1ms延迟

        info!("SDRAM初始化完成");
        Ok(())
    }

    pub fn write_buffer(&self, data: &[u8], addr: u32) {
        let base = 0xC000_0000;
        let phy_addr = base + addr;

        // 统一使用8位写入以匹配read_buffer
        for (i, &byte) in data.iter().enumerate() {
            unsafe {
                let target_addr = (phy_addr + i as u32) as *mut u8;
                target_addr.write_volatile(byte);
                // 移除调试日志以减少干扰
            }
        }
        // 在 write_buffer 和 read_buffer 中添加地址和数据验证
        info!("Writing {} bytes to address 0x{:x}", data.len(), addr);
    }

    pub fn read_buffer(&self, pbuf: &mut [u8], addr: u32, n: usize) {
        let base_addr = 0xC000_0000; // 与写入方法使用相同的基地址
        let start_addr = base_addr + addr; // 确保正确计算起始地址

        for i in 0..n {
            unsafe {
                let source_addr = (start_addr + i as u32) as *const u8;
                pbuf[i] = source_addr.read_volatile();
            }
        }
        info!("Reading {} bytes from address 0x{:x}", n, addr);
    }

    /// 写入u16数据到SDRAM
    pub fn write_u16(&self, data: u16, addr: u32) {
        let base = 0xC000_0000;
        let phy_addr = base + addr;

        unsafe {
            let target_addr = (phy_addr) as *mut u16;
            target_addr.write_volatile(data);
        }
        info!("Writing u16 0x{:04x} to address 0x{:x}", data, addr);
    }

    /// 从SDRAM读取u16数据
    pub fn read_u16(&self, addr: u32) -> u16 {
        let base = 0xC000_0000;
        let phy_addr = base + addr;

        let value = unsafe {
            let source_addr = (phy_addr) as *const u16;
            source_addr.read_volatile()
        };
        info!("Reading u16 0x{:04x} from address 0x{:x}", value, addr);
        value
    }

    /// 写入u32数据到SDRAM
    pub fn write_u32(&self, data: u32, addr: u32) {
        let base = 0xC000_0000;
        let phy_addr = base + addr;

        unsafe {
            let target_addr = (phy_addr) as *mut u32;
            target_addr.write_volatile(data);
        }
        info!("Writing u32 0x{:08x} to address 0x{:x}", data, addr);
    }

    /// 从SDRAM读取u32数据
    pub fn read_u32(&self, addr: u32) -> u32 {
        let base = 0xC000_0000;
        let phy_addr = base + addr;

        let value = unsafe {
            let source_addr = (phy_addr) as *const u32;
            source_addr.read_volatile()
        };
        info!("Reading u32 0x{:08x} from address 0x{:x}", value, addr);
        value
    }
}
