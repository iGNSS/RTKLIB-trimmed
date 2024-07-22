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
