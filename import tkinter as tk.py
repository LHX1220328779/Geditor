import tkinter as tk
from tkinter import filedialog, messagebox, ttk
import os

def select_csv():
    path = filedialog.askopenfilename(
        title="选择CSV文件",
        filetypes=[("CSV文件", "*.csv"), ("所有文件", "*.*")]
    )
    if path:
        csv_var.set(path)

def select_txt():
    path = filedialog.askopenfilename(
        title="选择TXT文件",
        filetypes=[("TXT文件", "*.txt"), ("所有文件", "*.*")]
    )
    if path:
        txt_var.set(path)

def process():
    csv_path = csv_var.get()
    txt_path = txt_var.get()

    if not csv_path or not os.path.isfile(csv_path):
        messagebox.showerror("错误", "请选择有效的CSV文件")
        return
    if not txt_path or not os.path.isfile(txt_path):
        messagebox.showerror("错误", "请选择有效的TXT文件")
        return

    try:
        segments_with_flag_1 = set()
        with open(txt_path, 'r', encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                parts = line.split()
                if len(parts) >= 2:
                    seg_name = parts[0]
                    flag = parts[1]
                    if flag == '1':
                        segments_with_flag_1.add(seg_name)

        if not segments_with_flag_1:
            messagebox.showinfo("提示", "TXT文件中没有均匀碾压开关为1的路段，无需修改。")
            return

        with open(csv_path, 'r', encoding='utf-8') as f:
            lines = f.readlines()

        header_fields = lines[0].strip().split(',')
        try:
            uniform_col = header_fields.index('均匀碾压')
        except ValueError:
            messagebox.showerror("错误", "CSV表头中未找到'均匀碾压'列")
            return

        modified_count = 0
        found_segments = set()

        for i, line in enumerate(lines):
            if i == 0:
                continue
            stripped = line.strip()
            if not stripped:
                continue
            fields = stripped.split(',')
            seg_name = fields[0]

            if seg_name in segments_with_flag_1:
                if uniform_col < len(fields):
                    current_val = fields[uniform_col]
                    if current_val == '0':
                        fields[uniform_col] = '1'
                        lines[i] = ','.join(fields) + '\n'
                        modified_count += 1
                    found_segments.add(seg_name)

        not_found_segments = segments_with_flag_1 - found_segments

        with open(csv_path, 'w', encoding='utf-8') as f:
            f.writelines(lines)

        msg = f"处理完成！\n\n修改了 {modified_count} 行的均匀碾压开关（0→1）。\n"
        if not_found_segments:
            msg += f"\n以下 {len(not_found_segments)} 个在TXT中标记为1的路段在CSV中未找到匹配行：\n"
            for s in sorted(not_found_segments):
                msg += f"  - {s}\n"

        messagebox.showinfo("完成", msg)

    except Exception as e:
        messagebox.showerror("错误", f"处理过程中出错：\n{e}")

root = tk.Tk()
root.title("均匀碾压开关设置工具")
root.resizable(True, True)

main_frame = ttk.Frame(root, padding=15)
main_frame.pack(fill=tk.BOTH, expand=True)

ttk.Label(main_frame, text="CSV文件：", font=("", 10)).grid(row=0, column=0, sticky=tk.W, pady=5)
csv_var = tk.StringVar()
ttk.Entry(main_frame, textvariable=csv_var, width=70).grid(row=0, column=1, padx=5, pady=5)
ttk.Button(main_frame, text="浏览...", command=select_csv).grid(row=0, column=2, padx=5, pady=5)

ttk.Label(main_frame, text="TXT文件：", font=("", 10)).grid(row=1, column=0, sticky=tk.W, pady=5)
txt_var = tk.StringVar()
ttk.Entry(main_frame, textvariable=txt_var, width=70).grid(row=1, column=1, padx=5, pady=5)
ttk.Button(main_frame, text="浏览...", command=select_txt).grid(row=1, column=2, padx=5, pady=5)

btn_frame = ttk.Frame(main_frame)
btn_frame.grid(row=2, column=0, columnspan=3, pady=15)
ttk.Button(btn_frame, text="执行", command=process, width=15).pack(side=tk.LEFT, padx=10)
ttk.Button(btn_frame, text="退出", command=root.destroy, width=15).pack(side=tk.LEFT, padx=10)

root.mainloop()