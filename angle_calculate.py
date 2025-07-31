import os
import json
import subprocess

root_dir = r"D:\evaluation\vehicle_tracking\New\a3"

# 三个程序的路径
program_a = r"D:\evaluation\vehicle_tracking\angle_calculate_1.py"
program_b = r"D:\evaluation\vehicle_tracking\angle_calculate_2.py"
program_c = r"D:\evaluation\vehicle_tracking\angle_calculate_3.py"

def run_program(program_path, video_dir):
    print(f"调用 {program_path} 处理文件夹 {video_dir}")
    # 通过subprocess调用python脚本，传递子文件夹路径作为第一个参数
    result = subprocess.run(["python", program_path, video_dir], capture_output=True, text=True)
    print(result.stdout)
    if result.stderr:
        print(f"错误信息：{result.stderr}")

def handle_video_dir(video_dir):
    for filename in sorted(os.listdir(video_dir)):
        if filename.endswith(".json"):
            json_path = os.path.join(video_dir, filename)
            try:
                with open(json_path, 'r') as f:
                    data = json.load(f)
                    camera = data.get("camera", {})
                    cam_x = camera.get("x", None)
                    cam_y = camera.get("y", None)

                    if cam_y is None:
                        print(f"{video_dir}: 无法读取 cam_y，跳过")
                        return
                    print(cam_y)
                    if  34 < cam_y <= 38:#右二
                        run_program(program_a, video_dir)
                    elif 29<= cam_y <= 34:#右三
                        run_program(program_b, video_dir)
                    elif 26 <= cam_y < 29:#最左车道
                        run_program(program_c, video_dir)
                    else:
                        print(f"{video_dir}: y={cam_y} 不在预设区间内，跳过")
                    return  # 找到一个有效json并处理后就退出函数
            except Exception as e:
                print(f"读取文件 {json_path} 时出错: {e}")
                return


def main():
    subdirs = [os.path.join(root_dir, d) for d in os.listdir(root_dir)
               if os.path.isdir(os.path.join(root_dir, d))]
    for video_dir in subdirs:
        handle_video_dir(video_dir)

if __name__ == "__main__":
    main()
