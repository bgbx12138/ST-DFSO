# encoding: utf-8
# @File  : curDataAutoDownload.py
# @Author: SuZhaoqiang
# @Date  : 2025/09/21/15:48
import copernicusmarine
from datetime import datetime, timedelta
import os
import shutil
import logging

def download_cur_data(parent_directory, start_date=None, days=7, area=None):
    # 设置日志记录
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

    # copernicusmarine.login()     # 登录, 下一行为登录配置文件


    # 获取起始日期（可参数化；默认今天）
    current_date = start_date or datetime.now().date()
    if area is None:
        area = {"lonMin": -180, "lonMax": 179.9169921875, "latMin": -80, "latMax": 90}
    # current_date = datetime(2025, 11, 25).date()  # 仅做测试，测试能否正常跨月

    # 创建子目录（格式：YYYYMMDD）
    sub_directory = current_date.strftime("%Y%m%d")
    output_dir = os.path.join(parent_directory, sub_directory)
    os.makedirs(output_dir, exist_ok=True)

    # 流场数据下载
    for i in range(days):
        # 计算目标日期
        target_date = current_date + timedelta(days=i)

        # 格式化日期字符串
        date_str = target_date.strftime("%Y-%m-%d")
        filename_date = target_date.strftime("%Y%m%d")

        # 设置时间范围（全天数据）
        start_time = f"{date_str}T00:00:00"
        end_time = f"{date_str}T00:00:00"

        # 构建输出文件路径
        output_filename = os.path.join(output_dir, f"{filename_date}00_cur.nc")

        logging.info(f"正在下载 {date_str} 的数据...")

        # 下载数据
        copernicusmarine.subset(
            dataset_id="cmems_mod_glo_phy-cur_anfc_0.083deg_P1D-m",
            variables=["uo", "vo"],
            minimum_longitude=area["lonMin"],
            maximum_longitude=area["lonMax"],
            minimum_latitude=area["latMin"],
            maximum_latitude=area["latMax"],
            start_datetime=start_time,
            end_datetime=end_time,
            minimum_depth=0.0,
            maximum_depth=1.0,   # 仅取表层（CMEMS 最上层 ≈0.494 m，只返回这一层）
            output_filename=output_filename,
            output_directory=output_dir
        )

        logging.info(f"已成功下载: {output_filename}")

    logging.info("所有数据下载尝试完成！")


# 保留文件自运行功能
if __name__ == "__main__":
    parent_directory = os.environ.get('ST_DFSO_DOWNLOAD_DIR', 'data/environment/raw')
    # 调用下载函数
    download_cur_data(parent_directory)
