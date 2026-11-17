# ST-DFSO

本仓库提供多日船舶航线规划算法ST-DFSO的代码、输入数据和全部810次实验结果，包含ACO、IAFSA、window ILS比较及五个机制消融版本。

[English](README.md) · [快速使用](docs/QUICKSTART_中文.md) · [结果说明](docs/RESULTS.md)

## 内容

- `src/main/java/`：航线搜索、对比算法及航程代价计算。
- `src/main/python/`：船舶响应模型和Java–Python接口。
- `config/`：六个场景、初始航线、参数及810组实验配置。
- `data/environment/`：论文所用环境数据，clone后无需另行下载。
- `data/model/`：模型输入和共用初始响应库。
- `data/results/`：810次运行记录、全部最终航线坐标及统计表。
- `analysis/`、`tools/`、`docs/`：统计复算、运行工具和使用说明。

## 安装和检查

实验运行环境为Windows、Java 11、Maven和Python 3.9.9。在仓库根目录执行：

```sh
python -m pip install -r requirements.txt
mvn package dependency:copy-dependencies -DskipTests
python tools/check_repository.py
python tools/check_model.py
python tools/check_environment.py --historical-hashes
```

## 复算论文统计结果

```sh
python analysis/reproduce_results.py --output outputs/paper-statistics
```

该命令读取已公开的810次结果，生成CPU预算比较、最终航线比较、跨目标统计及S6分解结果，不重新优化航线。结果位于所选目录的`data/`中，对应`data/results/tables/`中的六份PUBLICATION类CSV及分解表。输出目录必须尚不存在。

## 运行实验

单次实跑：

```sh
python tools/run_case.py --scenario S1 --objective TIME --algorithm ST-DFSO --output outputs/example --execute
```

安装试跑可再加`--smoke`，试跑结果不用于论文比较。完整批次：

```sh
python tools/run_frozen_experiments.py --check-only
python tools/run_frozen_experiments.py --output outputs/paper-experiments
```

完整批次执行810次优化：主比较360次、消融比较540行，其中90次完整算法运行共用。所有算法使用相同的441条共用初始航线响应，新响应计算和最终航线检查计入各次运行CPU。新运行的硬件与时间限制会影响结果；已公开数据用于精确复算论文统计。

原始项目材料遵循[非商业学术研究许可](LICENSE)，允许学术研究、复现和教学。商业使用需另行取得书面许可。第三方材料保留各自的许可条件。
