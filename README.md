English | [简体中文](https://github.com/Kevin-QAQ/RTKLIB-trimmed/blob/rtklib_2.4.3/README_zh-cn.md)

This project is forked from RTKLIB 2.4.3b, and its GUI ap rtknavi is changed to a pure C language project, which can be simulated by file input on Windows. This project cuts rtknavi to support only Real-Time Kinematic mode. This project can be debugged and run in both Visual Studio and VS Code.

## Requirements

### In Visual Studio

Open the .sln file of this project in Visual Studio and select Debug and x86 to debug and run.

### In VS Code

1. Support C99 standard 32-bit C language environment;
2. Open the task.json in the hidden folder .vscode, change "command": "C:\\msys64\\mingw32\\bin\\gcc.exe" to the directory path of your 32-bit gcc.exe;
3. Open the launch.json in the hidden folder .vscode and change "miDebuggerPath": "C:\\msys64\\mingw32\\bin\\gdb.exe" to the path of your own 32-bit gdb.exe;
4. Open c_cpp_properties.json in the hidden folder .vscode and change "compilerPath": "C:\\msys64\\mingw32\\bin\\gcc.exe" to the directory path of your own 32-bit gcc.exe;
5. Open any .c file, click 'Run and Debug' in the left column of VS Code, and click 'Start Debugging' button in the upper left corner (or press F5) to debug and run.

## RTKLIB 2.4.3b BUG

1. src/pntpos.c: rescode(): dion, dtrp, vion, vtrp are used uninitialised and can be initialised at declaration time:

```c
double dion=0, dtrp=0, vion=0, vtrp=0;
```

## References

[1]《GPS/GNSS原理与应用（第3版）》 Understanding GPS/GNSS Principles and Applications, Third Edition (Gnss Technology and Applications Series) (Elliott Kaplan, Christopher J. Hegarty) 

[2] Basics of the GPS Technique: Observation Equations, Geoffrey Blewitt

[3]《GPS原理与接收机设计（修订版）》谢钢 著，电子工业出版社

[4] [RTKLIB_manual_2.4.2.pdf](https://github.com/Kevin-QAQ/RTKLIB-trimmed/blob/rtklib_2.4.3/doc/manual_2.4.2.pdf)

[5] [https://github.com/LiZhengXiao99/Navigation-Learning/](https://github.com/LiZhengXiao99/Navigation-Learning/tree/main/01-RTKLIB%E6%BA%90%E7%A0%81%E9%98%85%E8%AF%BB)
