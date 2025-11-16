use defmt::info;
use stm32f4xx_hal::{pac, rcc::Rcc};

pub struct SdramDriver {
    fmc: pac::FMC,
}

impl SdramDriver {
    pub fn new(fmc: pac::FMC) -> Self {
        Self { fmc }
    }

    pub fn debug_status(&self) {
        let sdsr = self.fmc.sdsr().read().bits();
        info!("SDSR Status: 0x{:08X}", sdsr);
        info!("  - RE (Refresh Error): {}", (sdsr >> 0) & 1);
        info!("  - MODES1: {}", (sdsr >> 1) & 0b11);
        info!("  - MODES2: {}", (sdsr >> 3) & 0b11);
        info!("  - BUSY: {}", (sdsr >> 5) & 1);
    }

    pub fn debug_config(&self) {
        let sdcr = self.fmc.sdcr1().read().bits();
        info!("SDCR 配置: 0x{:08X}", sdcr);
        info!(
            "  地址位数: COL={}bit, ROW={}bit",
            (sdcr & 0b11) + 8,
            ((sdcr >> 2) & 0b11) + 11
        );
        info!(
            "  突发类型: {}/{}",
            if sdcr & (1 << 9) == 0 {
                "不使能"
            } else {
                "使能"
            },
            if sdcr & (1 << 12) == 0 {
                "非管线"
            } else {
                "管线"
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

        // 3. 修正控制寄存器（NC应该为1）
        let mut sdctrlreg: u32 = 0;
        sdctrlreg |= 0b01 << 0; // 修正：9位列地址 (NC=1)
        sdctrlreg |= 0b10 << 2; // 13位行地址 (NR=2)
        sdctrlreg |= 0b01 << 4; // 16位数据位宽 (MWID=1)
        sdctrlreg |= 1 << 6; // 4个内部存储区 (NB=1)
        sdctrlreg |= 0b10 << 7; // CAS延迟=2 (CAS=2)
        sdctrlreg |= 0 << 9; // 写保护禁用 (WP=0)
        sdctrlreg |= 0b10 << 10; // SDCLK = HCLK/2 (SDCLK=2)
        sdctrlreg |= 1 << 12; // 使能突发读 (RBURST=1)
        sdctrlreg |= 0 << 13; // 读通道延迟=0 (RPIPE=0)

        self.fmc.sdcr1().write(|w| unsafe { w.bits(sdctrlreg) });
        info!("控制寄存器配置完成: 0x{:08X}", sdctrlreg);

        // 2. 配置时序寄存器（关键修正）
        let mut sdtimereg: u32 = 0;
        sdtimereg |= 2 << 0; // TMRD=2周期
        sdtimereg |= 7 << 4; // TXSR=7周期
        sdtimereg |= 6 << 8; // TRAS=6周期
        sdtimereg |= 6 << 12; // TRC=6周期
        sdtimereg |= 2 << 16; // TWR=2周期
        sdtimereg |= 2 << 20; // TRP=2周期
        sdtimereg |= 2 << 24; // TRCD=2周期

        self.fmc.sdtr1().write(|w| unsafe { w.bits(sdtimereg) });
        info!("时序寄存器配置完成: 0x{:08X}", sdtimereg);

        // 3. SDRAM初始化序列
        info!("开始SDRAM初始化序列...");

        // 步骤1: 时钟配置使能
        self.send_cmd(0, 1, 0, 0)?;
        info!("步骤1: 时钟配置使能完成");

        // 延迟至少200us（重要！）
        cortex_m::asm::delay(500 * 180); // 168MHz系统时钟

        // 步骤2: 预充电所有存储区
        self.send_cmd(0, 2, 0, 0)?;
        cortex_m::asm::delay(100 * 180); // 100us延迟
        info!("步骤2: 预充电完成");

        // 步骤3: 自动刷新（执行8次）
        self.send_cmd(0, 3, 8, 0)?;
        cortex_m::asm::delay(100 * 180); // 增加延时
        info!("步骤3: 自动刷新完成");

        // 6. 加载模式寄存器
        let mut mregval: u16 = 0;
        mregval |= 0 << 0; // 突发长度=1
        mregval |= 0 << 3; // 突发类型=连续
        mregval |= 2 << 4; // CAS延迟=2
        mregval |= 0 << 7; // 操作模式=标准
        mregval |= 1 << 9; // 写突发模式=单点访问

        self.send_cmd(0, 4, 0, mregval)?;
        cortex_m::asm::delay(100 * 180); // 100us延迟
        info!("步骤4: 模式寄存器加载完成");

        // 5. 设置刷新定时器（更保守的值）
        let refresh_rate = 730;
        self.fmc.sdrtr().write(|w| unsafe { w.bits(refresh_rate) });
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
        let base_addr = 0xD000_0000;  // 修正基地址
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
