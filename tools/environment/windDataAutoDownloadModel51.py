# encoding: utf-8
# @File  : windDataAutoDownload.py
# @Author: SuZhaoqiang
# @Date  : 2025/09/22/10:48
import cdsapi
from datetime import datetime, timedelta
import os
import logging
import shutil
import calendar


def get_init_dates_for_target(target_date, max_months_back=3):
    """
    为目标日期生成可用的初始化日期列表（从近到远）
    最多回退max_months_back个月
    """
    init_dates = []

    # 从目标日期所在月份的1号开始
    current_year = target_date.year
    current_month = target_date.month

    for i in range(max_months_back):
        # 计算回退i个月的初始化日期
        if i == 0:
            # 当月1号
            init_year = current_year
            init_month = current_month
        else:
            # 回退i个月
            months_back = i
            init_year = current_year
            init_month = current_month - months_back

            while init_month <= 0:
                init_month += 12
                init_year -= 1

        # 创建初始化日期（每月的1号）
        init_date = datetime(init_year, init_month, 1).date()

        # 确保初始化日期不是未来日期（相对于今天）
        today = datetime.now().date()
        if init_date <= today:
            init_dates.append((init_year, init_month, init_date))

    return init_dates


def download_wind_data_model51(parent_directory, start_date=None, days=7, area=None):
    # 设置日志记录
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

    # 获取起始日期（可参数化；默认今天）
    current_date = start_date or datetime.now().date()
    if area is None:
        area = {"lonMin": -180, "lonMax": 179.9169921875, "latMin": -80, "latMax": 90}
    # current_date = datetime(2025, 11, 25).date()  # 测试用：2026年1月5日

    # 创建子目录（格式：YYYYMMDD）
    sub_directory = current_date.strftime("%Y%m%d")
    output_dir = os.path.join(parent_directory, sub_directory)
    os.makedirs(output_dir, exist_ok=True)

    # 初始化CDS客户端
    client = cdsapi.Client()

    # 循环下载未来 days 天的数据
    for day_offset in range(days):
        # 计算目标日期
        target_date = current_date + timedelta(days=day_offset)

        # 为目标日期生成初始化日期列表
        init_dates = get_init_dates_for_target(target_date, max_months_back=3)

        if not init_dates:
            logging.error(f"无法为 {target_date} 找到任何可用的初始化日期")
            continue

        # 格式化日期字符串用于文件名
        filename_date = target_date.strftime("%Y%m%d")

        # 下载一个时间点的数据：00
        for hour in ["00"]:
            output_filename = os.path.join(output_dir, f"{filename_date}{hour}_wind.nc")

            hour_value = int(hour)
            downloaded = False

            # 依次尝试不同的初始化日期
            for init_year, init_month, init_date in init_dates:
                # 计算leadtime（从初始化日期到目标日期指定小时的小时数）
                days_diff = (target_date - init_date).days
                leadtime = str(days_diff * 24 + hour_value)

                logging.info(
                    f"尝试下载 {target_date} {hour}时 的数据 (初始化: {init_year}-{init_month:02d}-01, leadtime: {leadtime})...")

                # 构建请求参数
                request = {
                    "originating_centre": "ecmwf",
                    "system": "51",
                    "variable": [
                        "10m_u_component_of_wind",
                        "10m_v_component_of_wind"
                    ],
                    "year": [str(init_year)],
                    "month": [str(init_month).zfill(2)],
                    "day": ["01"],  # 总是从初始化月1号开始
                    "leadtime_hour": [leadtime],
                    "data_format": "netcdf",
                    "area": [area["latMax"], area["lonMin"], area["latMin"], area["lonMax"]],
                }

                # 尝试下载数据
                try:
                    client.retrieve("seasonal-original-single-levels", request).download(output_filename)
                    logging.info(f"已成功下载: {output_filename}")
                    downloaded = True
                    break  # 成功则跳出循环
                except Exception as e:
                    error_msg = str(e)
                    logging.warning(f"使用初始化日期 {init_year}-{init_month:02d}-01 失败: {error_msg[:100]}")
                    # 继续尝试下一个初始化日期

            if not downloaded:
                logging.error(f"所有初始化日期尝试失败，无法下载 {target_date} {hour}时 的数据")

    logging.info("所有数据下载尝试完成！")


if __name__ == "__main__":
    parent_directory = os.environ.get('ST_DFSO_DOWNLOAD_DIR', 'data/environment/raw')
    # 调用下载函数
    download_wind_data_model51(parent_directory)
