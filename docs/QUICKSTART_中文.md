# 快速使用

在仓库根目录运行，测量实验使用Windows、Java 11、Maven和Python 3.9.9。

```sh
python -m pip install -r requirements.txt
mvn package dependency:copy-dependencies -DskipTests
python tools/check_repository.py
python tools/check_environment.py --historical-hashes
python tools/check_model.py
```

只复算论文统计，不重跑优化：

```sh
python analysis/reproduce_results.py --output outputs/paper-statistics
```

单次搜索：

```sh
python tools/run_case.py --scenario S1 --objective TIME --algorithm ST-DFSO --replicate 1 --output outputs/example --execute
```

加`--smoke`执行短试跑；不加`--execute`只显示配置。场景为S1–S6，目标可选TIME、ENERGY、MEAN_MOTION、EXPOSURE、COMPOSITE，算法可选ST-DFSO、ACO、IAFSA、ILS。ST-DFSO的`--variant`还可设为MINUS_LL、MINUS_AV、MINUS_FG、MINUS_ER或CORE。

完整运行：

```sh
python tools/run_frozen_experiments.py --check-only
python tools/run_frozen_experiments.py --output outputs/paper-experiments
```

中断后在同一命令末尾加`--resume`继续，保留同一输出目录和响应缓存。完整批次包含810次优化；输出目录必须独立于公开输入和源代码。单次运行输出航线坐标、五项目标值及CPU记录。试跑只用于安装检查，不能代替论文实验。
