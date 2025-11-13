# VisionFive2 (StarFive JH7110) SoC Drivers Overview

## SoC Architecture Overview
The **StarFive JH7110** is a quad-core RISC-V SoC that powers the VisionFive2 development board. It contains many specialized subsystems that require dedicated drivers.

## Core SoC Infrastructure Drivers

### 1. Clock Management (`drivers/clk/starfive/`)
```
├── clk-starfive-jh7110-aon.c     # Always-On clock domain
├── clk-starfive-jh7110-sys.c     # System clock domain  
├── clk-starfive-jh7110-stg.c     # Staging clock domain
├── clk-starfive-jh7110-isp.c     # ISP subsystem clocks
├── clk-starfive-jh7110-vout.c    # Video output clocks
├── clk-starfive-jh7110-pll.c     # PLL management
└── clk-starfive-jh7110-gen.c     # Generic clock utilities
```

**Purpose**: Manages the complex clock tree of the JH7110 SoC
- **AON Domain**: Ultra-low power clocks for always-on peripherals
- **SYS Domain**: Main system clocks for CPU, DDR, peripherals  
- **STG Domain**: Staging clocks for power management transitions
- **ISP Domain**: Camera and image processing pipeline clocks
- **VOUT Domain**: Display and video output clocks

### 2. Power Management (`drivers/soc/starfive/`)
```
└── jh7110_pmu.c                   # Power Management Unit driver
```

**Purpose**: Controls power domains and sleep states
- CPU power gating
- Peripheral power control
- Dynamic voltage/frequency scaling
- System suspend/resume

## Peripheral Drivers

### 3. SPI Controller (`drivers/spi/`)
```
└── spi-pl022-starfive.c          # ARM PL022 SPI with StarFive customizations
```

**Purpose**: High-speed SPI communication
- **Hardware**: ARM PL022 IP core with StarFive modifications
- **Features**: Master/slave mode, DMA support, multiple chip selects
- **Usage**: Flash storage, sensor communication, general SPI devices

### 4. Camera & ISP System (`drivers/media/platform/starfive/`)
```
├── v4l2_driver/
│   ├── stf_isp.c                 # Image Signal Processor driver
│   ├── stf_csi.c                 # Camera Serial Interface  
│   ├── stf_csiphy.c              # CSI PHY driver
│   ├── stf_vin.c                 # Video Input controller
│   ├── stf_video.c               # V4L2 video device interface
│   ├── stf_dvp.c                 # Digital Video Port
│   └── Camera sensor drivers:
│       ├── ov5640.c              # OmniVision 5MP sensor
│       ├── ov4689_mipi.c         # OmniVision 4MP MIPI sensor  
│       ├── ov13850_mipi.c        # OmniVision 13MP MIPI sensor
│       └── sc2235.c              # SmartSens 2MP sensor
```

**Purpose**: Complete camera pipeline
- **CSI**: MIPI Camera Serial Interface for high-speed camera data
- **ISP**: Hardware image processing (demosaic, noise reduction, etc.)
- **VIN**: Video input management and buffering
- **V4L2**: Standard Linux video interface

## DDR Memory Controller (U-Boot)

### 5. DDR Subsystem (`u-boot/drivers/ram/starfive/`)
```
├── starfive_ddr.c                # Main DDR controller driver
├── ddrphy_train.c                # DDR PHY training sequences
└── starfive_ddr.h                # DDR configuration structures
```

**Purpose**: Initialize and manage system memory
- **DDR4/LPDDR4 Support**: Multiple memory types
- **Training**: Automatic timing calibration
- **Frequencies**: 2133/2800/3200 MHz operation modes

## System Integration Drivers

### 6. GPIO & Pin Control
```
# Standard Linux GPIO framework with StarFive platform data
drivers/pinctrl/pinctrl-starfive-jh7110.c  (expected location)
drivers/gpio/gpio-starfive-jh7110.c        (expected location)
```

### 7. Ethernet Controllers
```
# GMAC Ethernet with StarFive platform glue
drivers/net/ethernet/stmicro/stmmac/dwmac-starfive.c
```

### 8. USB Controllers  
```
# USB 3.0 controllers with platform-specific setup
drivers/usb/dwc3/dwc3-starfive.c
```

### 9. PCIe Controller
```
# PCIe root complex driver
drivers/pci/controller/pcie-starfive.c
```

## Driver Architecture Patterns

### Clock Domain Separation
The JH7110 uses multiple independent clock domains:
```
AON (Always-On) ←→ SYS (System) ←→ STG (Staging) ←→ [ISP, VOUT]
```

### Power Management Integration
```
CPU ←→ PMU ←→ Clock Controllers ←→ Peripheral Drivers
```

### Video Pipeline
```
Camera Sensor → CSI PHY → CSI → ISP → VIN → V4L2 → Userspace
```

## Key Features

### Hardware Acceleration
- **ISP**: Real-time image processing
- **H.264/H.265**: Hardware video encode/decode
- **Neural Engine**: AI/ML acceleration
- **GPU**: 3D graphics acceleration

### High-Speed Interfaces  
- **PCIe 2.0**: M.2 storage, expansion cards
- **USB 3.0**: High-speed peripherals
- **GMAC**: Gigabit Ethernet
- **MIPI CSI**: High-resolution cameras

### Low Power Features
- **Power Domains**: Fine-grained power control
- **Clock Gating**: Dynamic power saving
- **DVFS**: Dynamic voltage/frequency scaling

## Development Considerations

### Device Tree Integration
All drivers rely heavily on device tree for platform configuration:
```dts
&i2c0 { /* Camera sensors */ };
&csi { /* Camera interface */ };
&isp { /* Image processing */ };
```

### V4L2 Media Framework
Camera subsystem uses complex media controller topology:
```
[Sensor] → [CSI PHY] → [CSI] → [ISP] → [Video Node]
```

### Clock Dependencies
Careful ordering required for clock domain initialization:
```
1. PLL setup
2. AON domain  
3. SYS domain
4. Peripheral domains (STG, ISP, VOUT)
```

This comprehensive driver ecosystem enables the VisionFive2 to function as a full-featured RISC-V development platform with multimedia capabilities.