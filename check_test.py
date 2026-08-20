import sys
import requests
import zipfile
import io
import pandas as pd
import matplotlib.pyplot as plt
import re  # 用于正则提取路段编码
from pathlib import Path

from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt, QDateTime
from PyQt5.QtWidgets import QDateTimeEdit, QFileDialog

BASE_URL = "http://10.198.92.39:8090"
TRUCK_URL = "http://10.198.92.34:8087/hbase_toexcel"
TRUCK_LIST_URL = "http://10.198.92.34:8087/extlog/truck_list"
REQUEST_TIMEOUT = (5, 60)


# 这些地址是内网服务，不能使用终端环境中的 HTTP(S)_PROXY。
# Session.trust_env=False 同时避开大小写形式的代理环境变量。
internal_session = requests.Session()
internal_session.trust_env = False


def get_route_list():
    r = internal_session.get(BASE_URL + "/api/route/history", timeout=REQUEST_TIMEOUT)
    r.raise_for_status()
    payload = r.json()
    if not isinstance(payload, dict) or not isinstance(payload.get("data"), list):
        raise ValueError("版本接口返回格式错误：缺少 data 列表")
    return payload["data"]


def get_truck_list():
    r = internal_session.get(TRUCK_LIST_URL, timeout=REQUEST_TIMEOUT)
    r.raise_for_status()
    payload = r.json()
    if not isinstance(payload, list):
        raise ValueError("车辆接口返回格式错误：应为列表")
    return payload


class MapViewer(QWidget):

    def __init__(self):

        super().__init__()

        self.setWindowTitle("Route Segment Viewer (自动读取模式)")

        layout = QVBoxLayout()

        # ===== 地图来源 =====
        layout.addWidget(QLabel("1. 选择地图加载方式"))
        self.map_source_box = QComboBox()
        self.map_source_box.addItem("从网站读取并下载", "website")
        self.map_source_box.addItem("从本地文件夹加载压缩包", "local")
        self.map_source_box.currentIndexChanged.connect(self.update_map_source_ui)
        layout.addWidget(self.map_source_box)

        # 网站地图控件
        self.website_label = QLabel("2. 选择网站地图版本")
        layout.addWidget(self.website_label)

        self.version_box = QComboBox()
        self.version_box.setEditable(True)
        self.version_box.setInsertPolicy(QComboBox.NoInsert)
        layout.addWidget(self.version_box)

        self.version_status = QLabel()
        self.version_status.setWordWrap(True)
        layout.addWidget(self.version_status)

        self.btn_reload_versions = QPushButton("重新读取网站地图版本")
        self.btn_reload_versions.clicked.connect(self.load_route_versions)
        layout.addWidget(self.btn_reload_versions)

        # 本地地图控件
        self.local_folder_label = QLabel("2. 选择地图压缩包所在文件夹")
        layout.addWidget(self.local_folder_label)

        self.map_folder_label = QLabel("未选择文件夹")
        self.map_folder_label.setWordWrap(True)
        layout.addWidget(self.map_folder_label)

        self.btn_choose_map_folder = QPushButton("选择地图文件夹")
        self.btn_choose_map_folder.clicked.connect(self.choose_map_folder)
        layout.addWidget(self.btn_choose_map_folder)

        self.local_archive_label = QLabel("勾选要同时加载的本地压缩包（默认全选）")
        layout.addWidget(self.local_archive_label)
        self.local_archive_list = QListWidget()
        self.local_archive_list.setMaximumHeight(120)
        layout.addWidget(self.local_archive_list)

        self.local_status = QLabel("请先选择地图文件夹")
        self.local_status.setWordWrap(True)
        layout.addWidget(self.local_status)

        self.btn_rescan_archives = QPushButton("重新扫描本地压缩包")
        self.btn_rescan_archives.clicked.connect(self.scan_map_archives)
        layout.addWidget(self.btn_rescan_archives)

        # ===== 加载地图按钮 =====
        btn_load = QPushButton("3. 加载地图数据")
        btn_load.clicked.connect(self.load_map)
        layout.addWidget(btn_load)

        # ===== 新增：选择TXT文件按钮 =====
        layout.addWidget(QLabel("4. 选择包含路段编码的TXT文件"))
        
        self.file_path_label = QLabel("未选择文件")
        self.file_path_label.setWordWrap(True)
        layout.addWidget(self.file_path_label)

        btn_choose_file = QPushButton("选择 TXT 文件")
        btn_choose_file.clicked.connect(self.choose_txt_file)
        layout.addWidget(btn_choose_file)

        # ===== 车辆与时间信息 =====
        layout.addWidget(QLabel("5. 车辆信息"))

        self.truck_input = QComboBox()
        self.truck_input.setEditable(True)
        self.truck_input.setInsertPolicy(QComboBox.NoInsert)
        self.truck_input.lineEdit().setPlaceholderText("请选择或输入车辆，例如 whks_22")
        layout.addWidget(self.truck_input)

        self.truck_status = QLabel()
        self.truck_status.setWordWrap(True)
        layout.addWidget(self.truck_status)

        layout.addWidget(QLabel("Start Time"))
        self.start_input = QDateTimeEdit()
        self.start_input.setDisplayFormat("yyyy-MM-dd HH:mm:ss")
        self.start_input.setDateTime(QDateTime.currentDateTime())
        self.start_input.setCalendarPopup(True)
        layout.addWidget(self.start_input)

        layout.addWidget(QLabel("End Time"))
        self.end_input = QDateTimeEdit()
        self.end_input.setDisplayFormat("yyyy-MM-dd HH:mm:ss")
        self.end_input.setDateTime(QDateTime.currentDateTime())
        self.end_input.setCalendarPopup(True)
        layout.addWidget(self.end_input)

        # ===== 绘图按钮 =====
        btn_draw = QPushButton("6. 自动绘制所有路段 + 车辆轨迹")
        btn_draw.setStyleSheet("font-size: 16px; font-weight: bold; color: blue;")
        btn_draw.clicked.connect(self.draw_segment)
        layout.addWidget(btn_draw)

        self.setLayout(layout)

        # ===== 初始化数据 =====
        self.map_folder = None
        self.route_data = []
        self.version_names = []
        self.version_ids = {}

        # 用于存储从TXT读取的路段
        self.segments_from_file = []
        # 用于存储地图数据
        self.loaded_zips = []
        self.segment_paths = {}

        self.local_archive_list.setEnabled(False)
        self.update_map_source_ui()
        self.load_route_versions(show_error_dialog=False)
        self.load_truck_list()

    def update_map_source_ui(self):
        """根据地图来源显示对应的一套控件。"""
        website_mode = self.map_source_box.currentData() == "website"
        website_widgets = (
            self.website_label,
            self.version_box,
            self.version_status,
            self.btn_reload_versions,
        )
        local_widgets = (
            self.local_folder_label,
            self.map_folder_label,
            self.btn_choose_map_folder,
            self.local_archive_label,
            self.local_archive_list,
            self.local_status,
            self.btn_rescan_archives,
        )
        for widget in website_widgets:
            widget.setVisible(website_mode)
        for widget in local_widgets:
            widget.setVisible(not website_mode)

    def load_route_versions(self, show_error_dialog=True):
        """加载网站地图版本。"""
        self.version_box.clear()
        self.version_box.setEnabled(False)
        self.version_status.setStyleSheet("color: #555;")
        self.version_status.setText("正在读取网站地图版本……")
        QApplication.processEvents()
        try:
            self.route_data = get_route_list()
            self.version_names = []
            self.version_ids = {}
            for route in self.route_data:
                version = str(route.get("version", "")).strip()
                route_id = route.get("id")
                if version and route_id is not None:
                    self.version_names.append(version)
                    self.version_ids[version] = route_id
            if not self.version_names:
                raise ValueError("网站没有返回有效地图版本")
            self.version_box.addItems(self.version_names)
            completer = QCompleter(self.version_names, self.version_box)
            completer.setCaseSensitivity(Qt.CaseInsensitive)
            completer.setFilterMode(Qt.MatchContains)
            self.version_box.setCompleter(completer)
            self.version_box.setEnabled(True)
            self.version_status.setStyleSheet("color: green;")
            self.version_status.setText(f"已读取 {len(self.version_names)} 个网站地图版本")
        except Exception as e:
            self.route_data = []
            self.version_names = []
            self.version_ids = {}
            message = f"网站地图版本读取失败：{e}"
            self.version_status.setStyleSheet("color: red;")
            self.version_status.setText(message)
            if show_error_dialog:
                QMessageBox.critical(self, "读取失败", message)

    def choose_map_folder(self):
        """选择地图目录，然后递归扫描目录中的 ZIP 文件。"""
        start_dir = str(self.map_folder or Path.home())
        folder = QFileDialog.getExistingDirectory(self, "选择地图压缩包所在文件夹", start_dir)
        if not folder:
            return

        self.map_folder = Path(folder)
        self.map_folder_label.setText(str(self.map_folder))
        self.scan_map_archives()

    def scan_map_archives(self):
        """扫描所选文件夹，并用文件名填充地图版本下拉框。"""
        self.local_archive_list.clear()
        self.local_archive_list.setEnabled(False)
        if not self.map_folder or not self.map_folder.is_dir():
            self.local_status.setStyleSheet("color: red;")
            self.local_status.setText("请先选择有效的地图文件夹")
            return

        archives = sorted(
            (path for path in self.map_folder.rglob("*") if path.is_file() and path.suffix.lower() == ".zip"),
            key=lambda path: str(path).lower(),
        )
        if not archives:
            self.local_status.setStyleSheet("color: red;")
            self.local_status.setText("该文件夹及其子文件夹中没有找到 ZIP 压缩包")
            return

        for path in archives:
            relative_path = path.relative_to(self.map_folder)
            item = QListWidgetItem(str(relative_path))
            item.setData(Qt.UserRole, str(path))
            item.setFlags(item.flags() | Qt.ItemIsUserCheckable)
            item.setCheckState(Qt.Checked)
            self.local_archive_list.addItem(item)

        self.local_archive_list.setEnabled(True)
        self.local_status.setStyleSheet("color: green;")
        self.local_status.setText(f"已找到 {len(archives)} 个本地压缩包")

    def load_truck_list(self):
        """加载轨迹服务实际支持的车辆，避免输入不存在的车辆简称。"""
        try:
            rows = get_truck_list()
            self.truck_values = {
                str(row.get("value", "")).strip()
                for row in rows
                if str(row.get("value", "")).strip()
            }
            for row in rows:
                name = str(row.get("name", "")).strip()
                value = str(row.get("value", "")).strip()
                if value:
                    self.truck_input.addItem(f"{name} ({value})", value)

            completer = QCompleter(sorted(self.truck_values), self.truck_input)
            completer.setCaseSensitivity(Qt.CaseInsensitive)
            completer.setFilterMode(Qt.MatchContains)
            self.truck_input.setCompleter(completer)
            self.truck_input.setCurrentIndex(-1)
            self.truck_status.setStyleSheet("color: green;")
            self.truck_status.setText(f"已读取 {len(self.truck_values)} 辆可用车辆")
        except Exception as e:
            self.truck_values = set()
            self.truck_status.setStyleSheet("color: #b06000;")
            self.truck_status.setText(f"车辆列表读取失败，可手工输入：{e}")

    def selected_truck(self):
        """取得车辆接口所需的 value，并兼容旧的 ks_XX 简称。"""
        index = self.truck_input.currentIndex()
        if index >= 0:
            value = self.truck_input.itemData(index)
            if value:
                return str(value)

        value = self.truck_input.currentText().strip().lower()
        if value.startswith("ks_"):
            whks_value = "wh" + value
            if not self.truck_values or whks_value in self.truck_values:
                value = whks_value
        return value

    def choose_txt_file(self):
        """选择TXT文件并解析路段编码"""
        file_name, _ = QFileDialog.getOpenFileName(self, "选择路段编码文件", ".", "Text Files (*.txt);;All Files (*)")
        
        if file_name:
            self.file_path_label.setText(file_name)
            try:
                with open(file_name, 'r', encoding='utf-8') as f:
                    content = f.read()
                    # 使用正则表达式提取可能的路段编码
                    # 假设编码包含字母、数字、下划线，且以字母开头，长度至少3位
                    # 你可以根据实际编码规则调整正则，例如 r'[A-Za-z_][\w_]+'
                    raw_segments = re.findall(r'\b\w+\b', content)
                    
                    # 简单过滤：排除纯数字或过短的项（根据实际业务调整）
                    # 这里假设路段编码通常包含下划线或者字母
                    self.segments_from_file = [s for s in raw_segments if len(s) > 2]
                    
                    # 去重
                    self.segments_from_file = list(set(self.segments_from_file))
                    
                    print(f"从文件解析到 {len(self.segments_from_file)} 个路段编码")
                    QMessageBox.information(self, "成功", f"成功读取文件，解析出 {len(self.segments_from_file)} 个路段编码。")
                    
            except Exception as e:
                print("读取文件失败:", e)
                QMessageBox.critical(self, "错误", f"读取文件失败: {str(e)}")

    def load_map(self):
        """网站加载一个 ZIP；本地模式同时合并所有已勾选 ZIP。"""
        opened_zips = []
        try:
            if self.map_source_box.currentData() == "website":
                version = self.version_box.currentText().strip()
                if version not in self.version_ids:
                    raise ValueError("请选择有效的网站地图版本")
                url = f"{BASE_URL}/api/route/download?id={self.version_ids[version]}"
                print("从网站下载:", url)
                r = internal_session.get(url, timeout=REQUEST_TIMEOUT)
                r.raise_for_status()
                zip_sources = [(f"网站版本：{version}", io.BytesIO(r.content))]
            else:
                selected_paths = []
                for row in range(self.local_archive_list.count()):
                    item = self.local_archive_list.item(row)
                    if item.checkState() == Qt.Checked:
                        selected_paths.append(Path(item.data(Qt.UserRole)))
                if not selected_paths:
                    raise ValueError("请至少勾选一个本地 ZIP 压缩包")
                zip_sources = [(f"本地文件：{path}", path) for path in selected_paths]

            segment_paths = {}
            skipped_sources = []
            duplicate_count = 0
            for source_name, source in zip_sources:
                try:
                    archive = zipfile.ZipFile(source, "r")
                    bad_file = archive.testzip()
                    if bad_file:
                        raise zipfile.BadZipFile(f"存在损坏文件：{bad_file}")

                    archive_segments = 0
                    for member in archive.namelist():
                        normalized = member.replace("\\", "/")
                        if "/segment_map/" in f"/{normalized}" and normalized.lower().endswith(".csv"):
                            segment_name = Path(normalized).stem
                            if segment_name in segment_paths:
                                duplicate_count += 1
                            segment_paths[segment_name] = (archive, member)
                            archive_segments += 1
                    if not archive_segments:
                        archive.close()
                        raise ValueError("没有 segment_map/*.csv 路段文件")
                    opened_zips.append(archive)
                except Exception as e:
                    skipped_sources.append(f"{source_name}：{e}")

            if not segment_paths:
                details = "\n".join(skipped_sources)
                raise ValueError(f"所选压缩包中没有可加载的路段文件\n{details}")

            for archive in self.loaded_zips:
                archive.close()
            self.loaded_zips = opened_zips
            self.segment_paths = segment_paths
            print("地图加载完成，识别到segment数量:", len(segment_paths))
            summary = (
                f"地图加载成功\n已加载 {len(opened_zips)} 个压缩包，"
                f"合并得到 {len(segment_paths)} 个路段"
            )
            if duplicate_count:
                summary += f"\n发现 {duplicate_count} 个同名路段，使用列表中靠后的压缩包"
            if skipped_sources:
                summary += f"\n跳过 {len(skipped_sources)} 个无效压缩包，详情见终端"
                print("跳过的压缩包:", *skipped_sources, sep="\n")
            QMessageBox.information(
                self,
                "成功",
                summary,
            )
        except Exception as e:
            for archive in opened_zips:
                archive.close()
            print("加载地图失败:", e)
            QMessageBox.critical(self, "错误", f"地图加载失败：{e}")

    def draw_segment(self):
        """核心绘图逻辑：直接使用文件中的路段列表"""
        
        # 1. 确定要绘制的路段列表
        # 优先使用从文件读取的，如果没读取到，则提示
        target_segments = self.segments_from_file
        
        if not target_segments:
            QMessageBox.warning(self, "提示", "请先选择并加载包含路段编码的TXT文件")
            return

        if not self.loaded_zips:
            QMessageBox.warning(self, "提示", "请先加载地图数据")
            return

        print(f"准备绘制 {len(target_segments)} 个路段...")

        segment_data = []
        missing_segments = []

        # 2. 遍历文件中的编码，从地图包中提取数据
        for seg in target_segments:
            if seg in self.segment_paths:
                archive, file_path = self.segment_paths[seg]
                try:
                    df = pd.read_csv(archive.open(file_path))
                    map_x = df.iloc[:, 0]
                    map_y = df.iloc[:, 1]
                    segment_data.append((seg, map_x, map_y))
                except Exception as e:
                    print(f"读取路段数据失败 {seg}: {e}")
            else:
                missing_segments.append(seg)

        if missing_segments:
            print(f"以下路段在地图包中未找到: {missing_segments[:5]}...") # 只打印前5个

        if not segment_data:
            QMessageBox.warning(self, "警告", "未在地图包中找到任何匹配的路段数据")
            return

        # 3. 请求车辆轨迹
        truck = self.selected_truck()
        if not truck:
            QMessageBox.warning(self, "警告", "请输入车辆编号")
            return

        if self.truck_values and truck not in self.truck_values:
            QMessageBox.warning(
                self,
                "车辆不存在",
                f"轨迹服务中不存在车辆“{truck}”，请从车辆下拉列表中选择。",
            )
            return

        start = self.start_input.dateTime().toString("yyyy-MM-dd HH:mm:ss")
        end = self.end_input.dateTime().toString("yyyy-MM-dd HH:mm:ss")

        params = {
            "truck": truck,
            "start": start,
            "end": end,
            "fields": "st_point_3d_x,st_point_3d_y"
        }

        try:
            r = internal_session.get(TRUCK_URL, params=params, timeout=REQUEST_TIMEOUT)
            r.raise_for_status()

            # 服务在业务失败时仍返回 HTTP 200，但正文是 {code: 500, message: ...}。
            if "application/json" in r.headers.get("content-type", "").lower():
                payload = r.json()
                raise ValueError(payload.get("message") or f"接口返回错误：{payload}")

            df_truck = pd.read_csv(io.StringIO(r.text))

            required_columns = ["st_point_3d_x", "st_point_3d_y"]
            missing_columns = [c for c in required_columns if c not in df_truck.columns]
            if missing_columns:
                raise ValueError(f"轨迹数据缺少坐标列：{', '.join(missing_columns)}")

            truck_xy = df_truck[required_columns].apply(pd.to_numeric, errors="coerce").dropna()
            if truck_xy.empty:
                raise ValueError("所选车辆在该时间范围内没有有效轨迹坐标")
            truck_x = truck_xy["st_point_3d_x"]
            truck_y = truck_xy["st_point_3d_y"]
            print(f"车辆 {truck} 获取到 {len(truck_xy)} 个有效轨迹点")
        except Exception as e:
            print("获取车辆轨迹失败:", e)
            QMessageBox.critical(self, "车辆轨迹获取失败", str(e))
            return

        # 4. 画图
        plt.figure(figsize=(10, 10))
        colors = ["blue", "cyan", "purple", "orange", "green", "brown", "magenta", "olive"]

        # 绘制路段
        for i, (seg, map_x, map_y) in enumerate(segment_data):
            plt.plot(
                map_x,
                map_y,
                color=colors[i % len(colors)],
                linewidth=1.5,
                alpha=0.7,
                label=seg
            )

        # 绘制车辆轨迹
        plt.plot(
            truck_x,
            truck_y,
            '.',
            markersize=3,
            color="red",
            label=f"Truck: {truck} ({len(truck_x)} points)"
        )

        plt.axis("equal")
        plt.title(f"Segments from File vs Truck Trajectory\nTotal Segments: {len(segment_data)}")
        plt.legend(loc='upper right', fontsize='small', ncol=2)
        plt.grid(True)
        plt.show()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = MapViewer()
    win.resize(400, 600)
    win.show()
    sys.exit(app.exec_())
