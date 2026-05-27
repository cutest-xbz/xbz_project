SET PATH=D:\Keil5_c51\ARM\ARMCC\Bin;D:\VMware\bin\;C:\Windows\system32;C:\Windows;C:\Windows\System32\Wbem;C:\Windows\System32\WindowsPowerShell\v1.0\;C:\Windows\System32\OpenSSH\;C:\Program Files (x86)\NVIDIA Corporation\PhysX\Common;C:\Program Files (x86)\Windows Kits\10\Windows Performance Toolkit\;C:\WINDOWS\system32;C:\WINDOWS;C:\WINDOWS\System32\Wbem;C:\WINDOWS\System32\WindowsPowerShell\v1.0\;C:\WINDOWS\System32\OpenSSH\;C:\Program Files\dotnet\;C:\WINDOWS\system32\downlevel;C:\Program Files\NVIDIA Corporation\NVIDIA app\NvDLISR;D:\matlab_download\runtime\win64;D:\matlab_download\bin;C:\Program Files (x86)\Windows Kits\8.1\Windows Performance Toolkit\;C:\Program Files (x86)\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility;D:\03. VS、Java、VScode、dev安装包及使用教程\vscode安装包及mingW\x86_64-15.2.0-release-win32-seh-ucrt-rt_v13-rev0\mingw64\bin;D:\x86_64-15.2.0-release-win32-seh-ucrt-rt_v13-rev0\mingw64\bin;D:\Matlab2025a\runtime\win64;D:\Matlab2025a\bin;C:\Program Files\nodejs\;C:\Program Files\PowerShell\7\;C:\Users\13188\AppData\Local\Microsoft\WindowsApps;D:\Microsoft VS Code\bin;D:\Quartus\Quartus2025.1\questa_fse\win64;D:\WorkBuddy\bin;C:\Users\13188\AppData\Roaming\npm;D:\Git\cmd;D:\Cursor\cursor\resources\app\bin;D:\Codex\node_modules\.bin
SET CPU_TYPE=STM32F103ZE
SET CPU_VENDOR=STMicroelectronics
SET UV2_TARGET=deepseek
SET CPU_CLOCK=0x00B71B00
"D:\Keil5_c51\ARM\ARMCC\Bin\ArmCC" --Via "..\out\core_cm3.__i"
"D:\Keil5_c51\ARM\ARMCC\Bin\ArmCC" --Via "..\out\system_stm32f10x.__i"
"D:\Keil5_c51\ARM\ARMCC\Bin\ArmAsm" --Via "..\out\startup_stm32f10x_hd._ia"
"D:\Keil5_c51\ARM\ARMCC\Bin\ArmCC" --Via "..\out\misc.__i"
"D:\Keil5_c51\ARM\ARMCC\Bin\ArmCC" --Via "..\out\stm32f10x_flash.__i"
"D:\Keil5_c51\ARM\ARMCC\Bin\ArmCC" --Via "..\out\stm32f10x_gpio.__i"
"D:\Keil5_c51\ARM\ARMCC\Bin\ArmCC" --Via "..\out\stm32f10x_rcc.__i"
"D:\Keil5_c51\ARM\ARMCC\Bin\ArmCC" --Via "..\out\stm32f10x_tim.__i"
"D:\Keil5_c51\ARM\ARMCC\Bin\ArmCC" --Via "..\out\stm32f10x_dma.__i"
"D:\Keil5_c51\ARM\ARMCC\Bin\ArmCC" --Via "..\out\stm32f10x_usart.__i"
"D:\Keil5_c51\ARM\ARMCC\Bin\ArmCC" --Via "..\out\stm32f10x_exti.__i"
"D:\Keil5_c51\ARM\ARMCC\Bin\ArmCC" --Via "..\out\device_timer.__i"
"D:\Keil5_c51\ARM\ARMCC\Bin\ArmCC" --Via "..\out\scheduler.__i"
"D:\Keil5_c51\ARM\ARMCC\Bin\ArmCC" --Via "..\out\time.__i"
"D:\Keil5_c51\ARM\ARMCC\Bin\ArmCC" --Via "..\out\motor.__i"
"D:\Keil5_c51\ARM\ARMCC\Bin\ArmCC" --Via "..\out\tracker.__i"
"D:\Keil5_c51\ARM\ARMCC\Bin\ArmCC" --Via "..\out\pwm.__i"
"D:\Keil5_c51\ARM\ARMCC\Bin\ArmCC" --Via "..\out\pid.__i"
"D:\Keil5_c51\ARM\ARMCC\Bin\ArmCC" --Via "..\out\car_drive.__i"
"D:\Keil5_c51\ARM\ARMCC\Bin\ArmCC" --Via "..\out\main.__i"
"D:\Keil5_c51\ARM\ARMCC\Bin\ArmLink" --Via "..\out\deepseek.lnp"
"D:\Keil5_c51\ARM\ARMCC\Bin\fromelf.exe" "..\out\deepseek.axf" --i32combined --output "..\out\deepseek.hex"
