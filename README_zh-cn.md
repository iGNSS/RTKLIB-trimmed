[English](https://github.com/Kevin-QAQ/RTKLIB-trimmed/blob/rtklib_2.4.3/README.md) | 简体中文

本工程分岔（fork）自 RTKLIB 2.4.3b，将其 GUI ap rtknavi 改为纯 C 语言的工程，可以在 Windows 上用文件输入的方式进行仿真。本工程对 rtknavi 进行了裁剪，仅支持 Real-Time Kinematic 模式。本工程可以在 Visual Studio 和 VS Code 中进行调试和运行。

## 要求

### 在 Visual Studio 中

使用 Visual Studio 打开本工程的 .sln 文件，选择 Debug 和 x86 即可调试和运行。

### 在 VS Code 中

1. 支持 C99 标准的 32 位 C 语言环境；
2. 打开隐藏文件夹 .vscode 中的 task.json，将其中的 "command": "C:\\msys64\\mingw32\\bin\\gcc.exe", 改为您自己的 32 位 gcc.exe 的目录路径；
3. 打开隐藏文件夹.vscode 中的 launch.json，将其中的 "miDebuggerPath": "C:\\msys64\\mingw32\\bin\\gdb.exe", 改为您自己的 32位 gdb.exe 的目录路径；
4. 打开隐藏文件夹 .vscode 中的 c_cpp_properties.json，将其中的 "compilerPath": "C:\\msys64\\mingw32\\bin\\gcc.exe", 改为您自己的 32 位 gcc.exe 的目录路径；
5. 打开任意 .c 文件，点击 VS Code 左栏的“运行和调试”，点击左上角的“开始调试”按钮（或按 F5）即可调试和运行。

## RTKLIB 2.4.3b BUG

1. src/pntpos.c: rescode(): dion, dtrp, vion, vtrp 未初始化就使用，可在声明时进行初始化：

```c
double dion=0, dtrp=0, vion=0, vtrp=0;
```
