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

## 参考文献

[1]《GPS/GNSS原理与应用（第3版）》 Understanding GPS/GNSS Principles and Applications, Third Edition (Gnss Technology and Applications Series) (Elliott Kaplan, Christopher J. Hegarty) 

[2] Basics of the GPS Technique: Observation Equations, Geoffrey Blewitt

[3]《GPS原理与接收机设计（修订版）》谢钢 著，电子工业出版社

[4] [RTKLIB_manual_2.4.2.pdf](https://github.com/Kevin-QAQ/RTKLIB-trimmed/blob/rtklib_2.4.3/doc/manual_2.4.2.pdf)

[5] [https://github.com/LiZhengXiao99/Navigation-Learning/](https://github.com/LiZhengXiao99/Navigation-Learning/blob/main/01-RTKLIB%E6%BA%90%E7%A0%81%E9%98%85%E8%AF%BB/07-RTKLIB%E6%BA%90%E7%A0%81%E9%98%85%E8%AF%BB%EF%BC%88%E4%B8%83%EF%BC%89%E5%AF%BC%E8%88%AA%E7%94%B5%E6%96%87%E3%80%81%E6%98%9F%E5%8E%86%E6%95%B0%E6%8D%AE%E8%AF%BB%E5%8F%96%E3%80%81%E5%8D%AB%E6%98%9F%E4%BD%8D%E7%BD%AE%E9%92%9F%E5%B7%AE%E8%AE%A1%E7%AE%97.md)
