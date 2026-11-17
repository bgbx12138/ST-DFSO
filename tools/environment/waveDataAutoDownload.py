# encoding: utf-8
# @File  : waveDataAutoDownload.py
# @Author: ZhaoZilong
# @Date  : 2025/09/21/16:05
import copernicusmarine
from datetime import datetime, timedelta
import os
import shutil
import logging

def download_wave_data(parent_directory, start_date=None, days=7, area=None):
    # 设置日志记录
    logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")

    # 设置凭证文件路径


    # 获取起始日期（可参数化；默认今天）
    today = start_date or datetime.today().date()
    if area is None:
        area = {"lonMin": -180, "lonMax": 179.9169921875, "latMin": -80, "latMax": 90}
    folder_name = today.strftime("%Y%m%d")
    end_date = today + timedelta(days=days)

    # 设置子目录
    sub_directory = today.strftime("%Y%m%d")
    output_dir = os.path.join(parent_directory, sub_directory)
    os.makedirs(output_dir, exist_ok=True)

    # 循环下载未来 days 天每天的数据
    for i in range(days):
        # 计算目标日期
        target_date = today + timedelta(days=i)

        # 为00时和12时分别下载数据
        for hour_suffix in ["00"]:
            # 构造下载时间（指定小时为00或12）
            download_time = f"{target_date.strftime('%Y-%m-%d')}T{hour_suffix}:00:00"

            # 构造输出文件名（文件名中包含小时后缀）
            output_filename = os.path.join(
                output_dir,
                f"{target_date.strftime('%Y%m%d')}{hour_suffix}_wave.nc"
            )

            # 下载文件
            copernicusmarine.subset(
                dataset_id="cmems_mod_glo_wav_anfc_0.083deg_PT3H-i",
                variables=["VHM0", "VTPK", "VMDR"],
                minimum_longitude=area["lonMin"],
                maximum_longitude=area["lonMax"],
                minimum_latitude=area["latMin"],
                maximum_latitude=area["latMax"],
                start_datetime=download_time,
                end_datetime=download_time,
                output_filename=output_filename
            )

            logging.info(f"已下载: {output_filename}")

    logging.info("所有文件下载完成！")

if __name__ == "__main__":
    parent_directory = os.environ.get('ST_DFSO_DOWNLOAD_DIR', 'data/environment/raw')
    # 调用下载函数
    download_wave_data(parent_directory)
