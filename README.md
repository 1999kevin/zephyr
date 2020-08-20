# zephyr
这个项目是用来测试zephyr的pwm，adc， gpio以及蓝牙模块等功能，其中BTtest，pwmtest，standbytest， voltagetest文件夹是在stm32_min_dev_blue(STM32F103C8T6)开发板上单独测试不同功能，app文件夹是将所有功能集合起来并在STM32f103CBT6开发板上运行。

## app
这个文件夹中的项目可以在STM32f103CBT6开发板上运行，主要内容包括STM32f103CBT6开发板在zephyrproject中的配置，针脚分配，设备树配置。demo功能为上电后开发板初始化，等待蓝牙的连接，蓝牙连接后，通过adc获取PA1针脚电压值，并通过UART2传给蓝牙透传出去。接下来系统开始轮询是否蓝牙是否收到PWM信息，收到后解码得到需要设置PWM的周期，脉冲，持续时间，并将PB10, PB11, PB6, PB7同时设置成相符的的PWM输出。

编译指令为‘west build -p auto -b stm32f103CBT6 -- -DBOARD_ROOT=<PATH_TO_app/board> ./app/ -DZEPHYR_TOOLCHAIN_VARIANT=gnuarmemb -DGNUARMEMB_TOOLCHAIN_PATH=<PATH_TO_TOOLCHAIN>‘

## BTtest
这个文件夹中的项目是用来单独测试蓝牙透传模块在stm32_min_dev_blue开发板上能否正常运行, 若成功，接收蓝牙会得到一串包含电压值的数据。

编译指令为‘west build -p auto -b stm32_min_dev_blue <PATH_TO_BTtest> -DZEPHYR_TOOLCHAIN_VARIANT=gnuarmemb -DGNUARMEMB_TOOLCHAIN_PATH=<PATH_TO_TOOLCHAIN>'

## pwmtest
这个文件夹中的项目是