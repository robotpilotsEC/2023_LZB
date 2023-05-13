SET PATH=E:\Keil5_Dowmload\ARM\ARM_Compiler_5.06u7\Bin;C:\Program Files (x86)\Common Files\Oracle\Java\javapath;C:\Program Files\MySQL\mysql\bin;C:\Windows\system32;C:\Windows;C:\Windows\System32\Wbem;C:\Windows\System32\WindowsPowerShell\v1.0\;C:\Windows\System32\OpenSSH\;C:\Program Files (x86)\NVIDIA Corporation\PhysX\Common;C:\Program Files\NVIDIA Corporation\NVIDIA NvDLISR;D:\Matlab\mat2019b\runtime\win64;D:\Matlab\mat2019b\bin;D:\Matlab\mat2019b\polyspace\bin;D:\nodejs\;D:\tesserract\Tesseract-OCR;D:\go\bin;C:\WINDOWS\system32;C:\WINDOWS;C:\WINDOWS\System32\Wbem;C:\WINDOWS\System32\WindowsPowerShell\v1.0\;C:\WINDOWS\System32\OpenSSH\;E:\mingw64\bin;E:\gitee\Git\cmd;C:\Users\26016\AppData\Local\Microsoft\WindowsApps;D:\Bandizip\;D:\Anaconda;D:\Anaconda\Scripts;D:\Anaconda\Library\mingw-w64\bin;D:\Anaconda\Library\bin;D:\VScode\Microsoft VS Code\bin;D:\nodejs\node_global;E:\Chrome_file;D:\java\jdk\bin;C:\Program Files\MySQL\MySQL Server 8.0\bin;D:\MySQL\MySQL Server 8.0\bin;D:\maven\apache-maven-3.6.3\bin;D:\DataGrip\DataGrip 2021.1.3\bin;C:\Users\26016\go\bin;C:\Users\26016\AppData\Local\Programs\oh-my-posh\bin;C:\;E:\OpenCV\bin;C:\Users\26016\AppData\Local\Programs\oh-my-posh\themes;C:\Users\26016\AppData\Local\GitHubDesktop\bin
SET CPU_TYPE=STM32F407VETx
SET CPU_VENDOR=STMicroelectronics
SET UV2_TARGET=Balance_Down
SET CPU_CLOCK=0x017D7840
"E:\Keil5_Dowmload\ARM\ARM_Compiler_5.06u7\Bin\ArmCC" --Via "balance_down\rp_math.__i"
"E:\Keil5_Dowmload\ARM\ARM_Compiler_5.06u7\Bin\ArmCC" --Via "balance_down\ave_filter.__i"
"E:\Keil5_Dowmload\ARM\ARM_Compiler_5.06u7\Bin\ArmCC" --Via "balance_down\kalman_filter.__i"
"E:\Keil5_Dowmload\ARM\ARM_Compiler_5.06u7\Bin\ArmCC" --Via "balance_down\crc.__i"
"E:\Keil5_Dowmload\ARM\ARM_Compiler_5.06u7\Bin\ArmLink" --Via "Balance_Down\Balance_Down.lnp"
"E:\Keil5_Dowmload\ARM\ARM_Compiler_5.06u7\Bin\fromelf.exe" "Balance_Down\Balance_Down.axf" --i32combined --output "Balance_Down\Balance_Down.hex"
