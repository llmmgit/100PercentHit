# 100 Percent Hit：YOLOv5 自定义模型训练、ONNX导出、Allwinner UINT8量化与Orange Pi Zero 3W部署指南

**文档版本：V2.0**  
**适用范围：YOLOv5n / YOLOv5s 单类别小球检测、Allwinner A733 NPU、Orange Pi Zero 3W、AR0234**  
**推荐正式模型：YOLOv5n**

---

## 1. 最终验证结论

本流程已完成以下实机验证：

- 自定义YOLOv5n和YOLOv5s均可完成训练、ONNX导出、UINT8量化和NBG导出。
- 使用Allwinner官方四步流程生成的模型可被Orange Pi Zero 3W上的VIPLite正常加载。
- YOLOv5n官方流程NBG约为1.5 MB，实机识别正常。
- YOLOv5s官方流程NBG约为5.3 MB，实机识别正常。
- 两个模型识别效果均较好，后续正式使用YOLOv5n。

> 重要：必须使用官方四步流程：`pegasus_import.sh → pegasus_quantize.sh → pegasus_inference.sh → pegasus_export_ovx.sh`。不要使用自定义一键`convert_export.sh`替代。

---

## 2. 环境与目录约定

### 2.1 Windows训练目录

以下使用通用占位目录，不包含个人信息：

```text
<PROJECT_ROOT>/yolov5/
├── train.py
├── val.py
├── export.py
├── yolov5n.pt
├── yolov5s.pt
├── dataset/
│   ├── data.yaml
│   ├── images/train
│   ├── images/val
│   ├── images/test
│   ├── labels/train
│   ├── labels/val
│   └── labels/test
└── runs/
```

`dataset/data.yaml`建议内容：

```yaml
path: dataset
train: images/train
val: images/val
test: images/test

names:
  0: ball
```

### 2.2 WSL与Docker

```text
WSL工作区：~/orangepi-npu/docker_data
Docker镜像：ubuntu-npu:v2.0.10.2
Docker容器：allwinner_npu
Docker共享目录：/workspace
Allwinner模型目录：/workspace/ai-sdk/models
```

### 2.3 Orange Pi Zero 3W

```text
项目目录：~/projects/100percenthit
正式模型目录：~/projects/100percenthit/yolo/models
AI SDK测试目录：~/projects/100percenthit/yolo_test/ai-sdk/examples
```

---

## 3. Windows训练YOLOv5模型

### 3.1 禁用W&B交互提示

在PowerShell中执行：

```powershell
$env:WANDB_DISABLED="true"
```

如仍不断提示，可卸载W&B：

```powershell
pip uninstall wandb -y
```

### 3.2 训练YOLOv5n

在官方YOLOv5仓库根目录执行：

```powershell
python train.py --weights yolov5n.pt --data dataset/data.yaml --img 640 --epochs 150 --batch-size 16 --device 0 --workers 0 --name ball_yolov5n
```

输出模型：

```text
runs/train/ball_yolov5n/weights/best.pt
```

### 3.3 训练YOLOv5s

```powershell
python train.py --weights yolov5s.pt --data dataset/data.yaml --img 640 --epochs 150 --batch-size 16 --device 0 --workers 0 --name ball_yolov5s
```

输出模型：

```text
runs/train/ball_yolov5s/weights/best.pt
```

### 3.4 独立test集验证

YOLOv5n：

```powershell
python val.py --weights runs/train/ball_yolov5n/weights/best.pt --data dataset/data.yaml --img 640 --task test --name test_yolov5n
```

YOLOv5s：

```powershell
python val.py --weights runs/train/ball_yolov5s/weights/best.pt --data dataset/data.yaml --img 640 --task test --name test_yolov5s
```

本轮实测结果：

| 模型 | Precision | Recall | mAP50 | mAP50-95 | PC推理时间 |
|---|---:|---:|---:|---:|---:|
| YOLOv5n | 0.952 | 0.951 | 0.992 | 0.679 | 3.2 ms |
| YOLOv5s | 0.984 | 0.978 | 0.992 | 0.715 | 4.4 ms |

YOLOv5s测试指标略高，但YOLOv5n速度和板端资源占用更有优势，实机识别效果满足要求，因此正式使用YOLOv5n。

---

## 4. 从best.pt导出四输出ONNX

### 4.1 必须使用传统官方YOLOv5输出结构

检查YOLOv5的`export.py`。Detect层导出设置应为：

```python
m.export = False
```

不要使用新版Ultralytics CLI生成的YOLOv5u模型。

### 4.2 导出YOLOv5n

```powershell
python export.py --weights runs/train/ball_yolov5n/weights/best.pt --img 640 --batch 1 --include onnx --opset 12
```

### 4.3 导出YOLOv5s

```powershell
python export.py --weights runs/train/ball_yolov5s/weights/best.pt --img 640 --batch 1 --include onnx --opset 12
```

将文件分别重命名为：

```text
best_5n.onnx
best_5s.onnx
```

### 4.4 ONNX结构检查

在安装了`onnx`的Python环境中执行：

```python
import onnx

for path in ["best_5n.onnx", "best_5s.onnx"]:
    model = onnx.load(path)
    print(path)
    for output in model.graph.output:
        dims = [d.dim_value for d in output.type.tensor_type.shape.dim]
        print(output.name, dims)
```

正确结构应为：

```text
output0 [1, 25200, 6]
onnx::Sigmoid_341 [1, 3, 80, 80, 6]
onnx::Sigmoid_376 [1, 3, 40, 40, 6]
onnx::Sigmoid_411 [1, 3, 20, 20, 6]
```

量化时只选择三个检测层输出：

```text
onnx::Sigmoid_341
onnx::Sigmoid_376
onnx::Sigmoid_411
```

不要使用动态输入，不要将NMS写入ONNX。

---

## 5. 准备WSL Docker环境

### 5.1 安装并启动Docker

```bash
sudo apt update
sudo apt install -y docker.io
sudo service docker start
sudo usermod -aG docker $USER
```

退出并重新进入WSL后检查：

```bash
docker info
```

### 5.2 导入Allwinner镜像

```bash
docker load -i ~/orangepi-npu/docker_image/ubuntu-npu_v2.0.10.2.tar
```

检查：

```bash
docker images
```

应看到：

```text
ubuntu-npu:v2.0.10.2
```

### 5.3 创建共享目录和容器

```bash
mkdir -p ~/orangepi-npu/docker_data

docker run -it \
  --name allwinner_npu \
  -v ~/orangepi-npu/docker_data:/workspace \
  ubuntu-npu:v2.0.10.2 \
  /bin/bash
```

后续进入已有容器：

```bash
docker start -ai allwinner_npu
```

### 5.4 初始化Allwinner环境

在Docker容器中执行：

```bash
cd /workspace/ai-sdk/models
source env.sh v3
```

检查：

```bash
echo $ACUITY_PATH
echo $VIV_SDK
```

本次验证环境：

```text
ACUITY_PATH=/root/acuity-toolkit-whl-6.30.22/bin
VIV_SDK=/root/Vivante_IDE/VivanteIDE5.11.0/cmdtools
```

---

## 6. 复制ONNX和校准图片到WSL

### 6.1 复制ONNX

在Windows PowerShell执行，将`<WSL_DISTRO>`替换为实际WSL名称：

```powershell
Copy-Item "runs\train\ball_yolov5n\weights\best_5n.onnx" "\\wsl$\<WSL_DISTRO>\home\<WSL_USER>\orangepi-npu\docker_data\best_5n.onnx"
Copy-Item "runs\train\ball_yolov5s\weights\best_5s.onnx" "\\wsl$\<WSL_DISTRO>\home\<WSL_USER>\orangepi-npu\docker_data\best_5s.onnx"
```

### 6.2 准备校准集

从训练集选择约100张AR0234真实图片，覆盖：

- 静态和飞行小球
- 远近不同尺寸
- 清晰和运动模糊
- 不同画面位置
- 无球负样本
- 头部、顶灯、膝盖等容易误检的场景

将图片放到：

```text
~/orangepi-npu/docker_data/calibration_images
```

容器内检查：

```bash
find /workspace/calibration_images -type f -iname "*.jpg" | wc -l
```

---

## 7. YOLOv5n官方UINT8量化流程

以下为最终验证成功的标准流程。

### 7.1 建立全新模型目录

在Docker容器中执行：

```bash
cd /workspace/ai-sdk/models

rm -rf yolov5n_ball_official
mkdir -p yolov5n_ball_official/images

cp /workspace/best_5n.onnx \
  yolov5n_ball_official/yolov5n_ball_official.onnx

cp /workspace/calibration_images/*.jpg \
  yolov5n_ball_official/images/
```

生成`inputs_outputs.txt`：

```bash
cat > yolov5n_ball_official/inputs_outputs.txt <<'EOF'
--inputs images --input-size-list '3,640,640' --outputs 'onnx::Sigmoid_341 onnx::Sigmoid_376 onnx::Sigmoid_411'
EOF
```

生成`channel_mean_value.txt`：

```bash
cat > yolov5n_ball_official/channel_mean_value.txt <<'EOF'
0 0 0 0.00392157
EOF
```

生成校准集列表。必须在模型目录内使用`images/...`相对路径：

```bash
cd /workspace/ai-sdk/models/yolov5n_ball_official
find images -type f -iname "*.jpg" | sort > dataset.txt
head dataset.txt
wc -l dataset.txt
```

正确示例：

```text
images/ball_100001.jpg
images/ball_100002.jpg
```

错误示例：

```text
yolov5n_ball_official/images/ball_100001.jpg
```

错误路径会导致量化阶段提示图片找不到。

### 7.2 复制官方脚本并初始化环境

```bash
cd /workspace/ai-sdk/models
source env.sh v3
cp ../scripts/* .
chmod +x pegasus_*.sh
```

### 7.3 官方步骤一：导入ONNX

```bash
./pegasus_import.sh yolov5n_ball_official
```

成功标志：

```text
Error(0),Warning(0)
```

导入后应生成：

```text
yolov5n_ball_official.data
yolov5n_ball_official.json
yolov5n_ball_official_inputmeta.yml
yolov5n_ball_official_postprocess_file.yml
```

### 7.4 修改输入缩放

执行：

```bash
cd /workspace/ai-sdk/models/yolov5n_ball_official

sed -i '/scale:/,/preproc_node_params:/ {
  s/- 1.0/- 0.00392157/g
}' yolov5n_ball_official_inputmeta.yml
```

检查：

```bash
grep -A6 -n "scale:" yolov5n_ball_official_inputmeta.yml
```

必须看到：

```yaml
scale:
- 0.00392157
- 0.00392157
- 0.00392157
```

不要破坏YAML列表结构。

### 7.5 官方步骤二：UINT8量化

```bash
cd /workspace/ai-sdk/models
source env.sh v3
./pegasus_quantize.sh yolov5n_ball_official uint8
```

成功标志：

```text
End quantization
Error(0),Warning(0)
SUCCESS
```

### 7.6 官方步骤三：量化仿真

```bash
cd /workspace/ai-sdk/models
source env.sh v3
./pegasus_inference.sh yolov5n_ball_official uint8 2>&1 \
  | tee yolov5n_ball_official_inference.log
```

成功标志：

```text
End inference
Error(0),Warning(0)
```

### 7.7 官方步骤四：导出NBG

```bash
cd /workspace/ai-sdk/models
source env.sh v3
./pegasus_export_ovx.sh yolov5n_ball_official uint8
```

成功标志：

```text
End exporting ovxlib case
Error(0),Warning(0)
```

最终文件：

```text
/workspace/ai-sdk/models/yolov5n_ball_official/wksp/yolov5n_ball_official_uint8_nbg_unify/network_binary.nb
```

复制到共享目录：

```bash
cp \
/workspace/ai-sdk/models/yolov5n_ball_official/wksp/yolov5n_ball_official_uint8_nbg_unify/network_binary.nb \
/workspace/yolov5n_ball_official_uint8.nb
```

检查：

```bash
ls -lh /workspace/yolov5n_ball_official_uint8.nb
```

本次成功模型大小：约`1.5 MB`。

---

## 8. YOLOv5s官方UINT8量化流程

如需生成5s模型，执行同样流程，仅替换模型名称。

### 8.1 建立目录

```bash
cd /workspace/ai-sdk/models

rm -rf yolov5s_ball_official
mkdir -p yolov5s_ball_official/images

cp /workspace/best_5s.onnx \
  yolov5s_ball_official/yolov5s_ball_official.onnx

cp /workspace/calibration_images/*.jpg \
  yolov5s_ball_official/images/

cat > yolov5s_ball_official/inputs_outputs.txt <<'EOF'
--inputs images --input-size-list '3,640,640' --outputs 'onnx::Sigmoid_341 onnx::Sigmoid_376 onnx::Sigmoid_411'
EOF

cat > yolov5s_ball_official/channel_mean_value.txt <<'EOF'
0 0 0 0.00392157
EOF

cd yolov5s_ball_official
find images -type f -iname "*.jpg" | sort > dataset.txt
```

### 8.2 执行官方四步

```bash
cd /workspace/ai-sdk/models
source env.sh v3

./pegasus_import.sh yolov5s_ball_official
```

修改scale：

```bash
cd /workspace/ai-sdk/models/yolov5s_ball_official
sed -i '/scale:/,/preproc_node_params:/ {
  s/- 1.0/- 0.00392157/g
}' yolov5s_ball_official_inputmeta.yml
```

继续执行：

```bash
cd /workspace/ai-sdk/models
./pegasus_quantize.sh yolov5s_ball_official uint8
./pegasus_inference.sh yolov5s_ball_official uint8 2>&1 \
  | tee yolov5s_ball_official_inference.log
./pegasus_export_ovx.sh yolov5s_ball_official uint8
```

最终文件：

```text
/workspace/ai-sdk/models/yolov5s_ball_official/wksp/yolov5s_ball_official_uint8_nbg_unify/network_binary.nb
```

复制：

```bash
cp \
/workspace/ai-sdk/models/yolov5s_ball_official/wksp/yolov5s_ball_official_uint8_nbg_unify/network_binary.nb \
/workspace/yolov5s_ball_official_uint8.nb
```

本次成功模型大小：约`5.3 MB`。

---

## 9. 关于转换过程中的紫色Warning

转换过程中可能出现紫色或醒目的警告，例如：

- 找不到`libcudart.so`
- TensorFlow没有配置GPU
- TensorFlow AutoGraph无法转换某些内部对象
- CPU指令优化提示

在本项目的Docker CPU转换环境中，这类信息可能正常出现。

判断成功与否不要只看文字颜色，应以每一步最终结果为准：

```text
Error(0),Warning(0)
```

并确认对应文件确实生成：

- 导入：`.json`、`.data`、`inputmeta.yml`、`postprocess_file.yml`
- 量化：`*_uint8.quantize`
- 仿真：`inf/`下生成tensor
- 导出：`network_binary.nb`

如果最终出现Pegasus参数错误、`ERROR`、缺少模型文件或未生成`network_binary.nb`，则不能视为成功。

---

## 10. 不要使用的错误路线

### 10.1 不要使用一键`convert_export.sh`

本轮曾使用自定义：

```bash
./convert_export.sh <model> uint8 t527
```

生成的文件大小为：

- 5n约2.1 MB
- 5s约6.2 MB

这两个模型在3W上出现`Failed to initialize YOLO model`，不可使用。

### 10.2 不要把`--pack-nbg-unify`改成`--pack-nbg-viplite`

当前Pegasus版本不识别：

```text
--pack-nbg-viplite
```

会报：

```text
pegasus: error: unrecognized arguments: --pack-nbg-viplite
```

官方成功流程保持脚本原状，最终目录名为：

```text
*_uint8_nbg_unify
```

该目录中的`network_binary.nb`已在VIPLite 2.0.3.2驱动上实机验证可加载。

### 10.3 不要复用旧模型生成的inputmeta和postprocess文件

应在每次执行`pegasus_import.sh`后使用新生成的：

```text
<model>_inputmeta.yml
<model>_postprocess_file.yml
```

仅修改新生成`inputmeta.yml`中的三通道scale。

---

## 11. 从WSL导出模型到Windows

退出Docker容器：

```bash
exit
```

在WSL中复制5n模型到Windows目标目录：

```bash
cp ~/orangepi-npu/docker_data/yolov5n_ball_official_uint8.nb \
"/mnt/c/<WINDOWS_TARGET_DIR>/yolov5n_ball_official_uint8.nb"
```

如需5s：

```bash
cp ~/orangepi-npu/docker_data/yolov5s_ball_official_uint8.nb \
"/mnt/c/<WINDOWS_TARGET_DIR>/yolov5s_ball_official_uint8.nb"
```

---

## 12. Orange Pi Zero 3W部署

### 12.1 先备份旧模型

```bash
cd ~/projects/100percenthit/yolo/models

cp yolov5n_ball_uint8.nb yolov5n_ball_uint8.nb.bak
cp yolov5s_ball_uint8.nb yolov5s_ball_uint8.nb.bak
```

先使用临时文件名上传：

```text
yolov5n_ball_official_uint8.nb
yolov5s_ball_official_uint8.nb
```

### 12.2 替换正式5n模型

停止服务：

```bash
sudo systemctl stop percenthit
```

替换：

```bash
cd ~/projects/100percenthit/yolo/models
cp yolov5n_ball_official_uint8.nb yolov5n_ball_uint8.nb
```

检查文件：

```bash
ls -lh yolov5n_ball_uint8.nb
```

5n官方流程模型应约为`1.5 MB`。

### 12.3 启动并检查服务

```bash
sudo systemctl start percenthit
sudo systemctl status percenthit --no-pager
```

查看模型加载错误：

```bash
sudo journalctl -u percenthit -b -n 200 --no-pager \
  | grep -Ei "error|failed|exception|traceback|yolo|npu|viplite"
```

成功时不应出现：

```text
RuntimeError: Failed to initialize YOLO model
```

### 12.4 模型路径配置

正式5n配置：

```yaml
vision:
  detector: yolo
  yolo_model: yolo/models/yolov5n_ball_uint8.nb
```

当前项目网页保存配置统一写入`config/local.yaml`。

### 12.5 回退旧模型

如果新模型无法加载：

```bash
sudo systemctl stop percenthit
cd ~/projects/100percenthit/yolo/models
cp -f yolov5n_ball_uint8.nb.bak yolov5n_ball_uint8.nb
sudo systemctl start percenthit
```

---

## 13. Orange Pi官方示例后处理适配

对于单类别`ball`模型，官方YOLOv5 COCO示例需要适配：

- `cls_num = 80`改为`cls_num = 1`
- 每个预测由85个值改为6个值
- 类别ID由COCO的32改为0
- 类别名称改为`ball`
- 三输出缓冲区末端范围使用`size`，不要使用`size - 1`

三个输出尺寸：

```text
1 × 3 × 80 × 80 × 6
1 × 3 × 40 × 40 × 6
1 × 3 × 20 × 20 × 6
```

建议保留独立目录：

```text
ai-sdk/examples/yolov5_ball_custom
```

不要直接破坏官方`yolov5`示例。

---

## 14. 最终文件保留清单

### Windows保留

```text
best_5n.pt
best_5s.pt
best_5n.onnx
best_5s.onnx
dataset/
```

### WSL共享目录保留

```text
/workspace/best_5n.onnx
/workspace/best_5s.onnx
/workspace/yolov5n_ball_official_uint8.nb
/workspace/yolov5s_ball_official_uint8.nb
/workspace/calibration_images/
/workspace/ai-sdk/
```

### Orange Pi保留

```text
~/projects/100percenthit/yolo/models/yolov5n_ball_uint8.nb
~/projects/100percenthit/yolo/models/yolov5n_ball_uint8.nb.bak
~/projects/100percenthit/yolo/models/yolov5s_ball_uint8.nb
~/projects/100percenthit/yolo/models/yolov5s_ball_uint8.nb.bak
```

---

## 15. 安全清理

确认官方模型已在3W验证成功后，可删除错误的一键导出模型：

```bash
rm -f /workspace/yolov5n_ball_new_uint8.nb
rm -f /workspace/yolov5s_ball_new_uint8.nb
```

可删除错误的一键量化工程：

```bash
rm -rf /workspace/ai-sdk/models/yolov5n_ball_new
rm -rf /workspace/ai-sdk/models/yolov5s_ball_new
rm -f /workspace/ai-sdk/models/yolov5n_ball_new*.log
rm -f /workspace/ai-sdk/models/yolov5s_ball_new*.log
```

官方工程建议保留：

```text
/workspace/ai-sdk/models/yolov5n_ball_official
/workspace/ai-sdk/models/yolov5s_ball_official
```

如需释放空间，可删除仿真输出，但保留最终NBG目录：

```bash
cd /workspace/ai-sdk/models
rm -rf yolov5n_ball_official/inf
rm -rf yolov5s_ball_official/inf

find yolov5n_ball_official/wksp -mindepth 1 -maxdepth 1 \
  ! -name 'yolov5n_ball_official_uint8_nbg_unify' -exec rm -rf {} +

find yolov5s_ball_official/wksp -mindepth 1 -maxdepth 1 \
  ! -name 'yolov5s_ball_official_uint8_nbg_unify' -exec rm -rf {} +
```

---

## 16. 最短复现清单

后续重新训练新模型时，按以下顺序执行：

1. 官方YOLOv5训练得到`best.pt`。
2. 设置`m.export = False`。
3. 使用`export.py`导出四输出ONNX。
4. 检查三个检测层输出名和形状。
5. 从真实AR0234训练图片中选择约100张校准图。
6. 在Docker中创建全新模型目录。
7. 生成`inputs_outputs.txt`、`channel_mean_value.txt`和相对路径`dataset.txt`。
8. `pegasus_import.sh`导入。
9. 将新生成`inputmeta.yml`的三个scale改为`0.00392157`。
10. `pegasus_quantize.sh`量化。
11. `pegasus_inference.sh`仿真。
12. `pegasus_export_ovx.sh`导出。
13. 从`*_uint8_nbg_unify/network_binary.nb`复制最终模型。
14. 先用临时名称上传3W。
15. 备份旧模型后替换正式模型。
16. 重启服务并检查日志。
17. 实测识别、漏检、误检和连续运行稳定性。

---

## 17. 当前正式选择

正式模型使用：

```text
YOLOv5n UINT8 NBG
```

理由：

- 真实场景识别效果良好。
- 板端成功加载并稳定运行。
- 模型文件约1.5 MB。
- 相比YOLOv5s具有更低的NPU推理开销。
- 更适合Orange Pi Zero 3W上的实时视觉伺服链路。
