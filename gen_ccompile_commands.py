import os
import sys
import json
import subprocess
import shutil
import re

# 如果存在编译器绝对路径找不到的问题，可在此强制指定交叉编译器的 bin 目录
CROSS_COMPILER_PATH = r"D:\SoftWare_Work\RT-ThreadStudio\repo\Extract\ToolChain_Support_Packages\ARM\GNU_Tools_for_ARM_Embedded_Processors\5.4.1\bin"

def find_make_cmd():
    """在 Windows 寻找可用的 make 命令"""
    candidates = ['make', 'mingw32-make']
    for cmd in candidates:
        if shutil.which(cmd):
            return cmd
    return None

def main():
    print("开始生成 compile_commands.json (Makefile/Eclipse 版)...")
    
    current_dir = os.path.abspath(os.getcwd())
    debug_dir = os.path.join(current_dir, 'Debug')
    
    if not os.path.exists(debug_dir) or not os.path.exists(os.path.join(debug_dir, 'makefile')):
        print(f"错误: 在 {debug_dir} 中找不到 makefile。")
        print("请确保您在 RT-Thread Studio/Eclipse 工程根目录执行此脚本，并且工程已经生成过 Makefile (通常位于 Debug 目录下)。")
        sys.exit(1)

    make_cmd = find_make_cmd()
    if not make_cmd:
        print("错误: 在系统 PATH 中找不到 'make' 或 'mingw32-make'。")
        sys.exit(1)

    print(f"找到 Make 工具: {make_cmd}")
    
    # 获取构建输出
    # 添加 --always-make 选项 (也可以用 -B) 强制 Make 干运行所有未修改的依赖
    print(f"正在全解析编译日志 ({make_cmd} --always-make -n all) ...")
    build_proc = subprocess.run([make_cmd, "--always-make", "-n", "all"], 
                            cwd=debug_dir, 
                            stdout=subprocess.PIPE, 
                            stderr=subprocess.PIPE, 
                            text=True,
                            encoding='utf-8', 
                            errors='replace')

    if build_proc.returncode != 0 and not build_proc.stdout:
         print(f"警告: '{make_cmd} -n' 运行失败，且没有输出。请检查 Makefile 或工具链配置。")
         sys.exit(1)

    lines = build_proc.stdout.splitlines()
    compile_commands = []
    
    # 编译器命令正则表达式，支持带有路径的或直接的 arm-none-eabi-gcc 等
    compiler_pattern = re.compile(
        r'^\s*(?:.*?[\/\\])?(?:arm-\S+-gcc|arm-\S+-g\+\+|gcc|g\+\+|cc|c\+\+|clang|clang\+\+)(?:\.exe)?\s'
    )
    
    # 提取源文件的正则表达式，查找例如 "../applications/main.c" 或类似结构
    # 通常 Makefile 里编译文件的最后几个参数包含源文件，尤其是以 .c / .cpp 结尾的字符串
    file_pattern = re.compile(r'(?:^|\s)(["\']?[^\s]+\.(?:c|cpp|cc|cxx|s|S)["\']?)(?:\s|$)')

    # Makefile 在 Eclipse/RT-Thread 工程里通常在 Debug 目录运行
    active_dir = debug_dir

    for line in lines:
        line = line.strip()
        if not line:
            continue
            
        # 排除链接目标 (包含 -o "rtthread.elf" 以及很多 .o 文件，通常不会包含 -c)
        if compiler_pattern.search(line) and "-c " in line:
            if CROSS_COMPILER_PATH:
                # 替换为带绝对路径的编译器指令，确保使用反斜杠，并避免 re.sub 遇到反斜杠转义
                line = re.sub(r'(^|\s)(arm-\S+-(?:gcc|g\+\+)(?:\.exe)?)(?=\s)', lambda m: f"{m.group(1)}{CROSS_COMPILER_PATH}\\{m.group(2)}", line)
                
            # 找到所有的类文件路径后缀匹配，最后一个通常就是源文件
            matches = file_pattern.findall(line)
            if matches:
                 # 遍历匹配项，寻找不是 .o 也不是 .d 的真正源文件
                src_file = None
                for match in reversed(matches):
                    clean_match = match.strip('"\'')
                    if clean_match.endswith(('.c', '.cpp', '.cc', '.cxx', '.s', '.S')):
                        src_file = clean_match
                        break
                
                if src_file:
                    # 处理路径：Make 打印的可能是相对 Debug 目录的路径（比如 ../applications/main.c）
                    if os.path.isabs(src_file):
                        abs_src_file = os.path.normpath(src_file)
                    else:
                        abs_src_file = os.path.abspath(os.path.join(active_dir, src_file))
                        
                    cmd_dict = {
                        "directory": active_dir,
                        "command": line,
                        "file": abs_src_file
                    }
                    compile_commands.append(cmd_dict)

    # 去重
    unique_commands = { (cmd["file"], cmd["command"]): cmd for cmd in compile_commands }
    compile_commands = list(unique_commands.values())

    if not compile_commands:
        print("警告: 运行 make 成功，但没有找到有效的 C/C++ 编译命令！")
        print("请检查 Makefile 结构。")
    else:
        out_path = os.path.join(current_dir, 'compile_commands.json')
        with open(out_path, 'w', encoding='utf-8') as f:
            json.dump(compile_commands, f, indent=4)
        print(f"成功生成 compile_commands.json，共包含 {len(compile_commands)} 个编译选项。")
        print(f"输出位置: {out_path}")

if __name__ == "__main__":
    main()
