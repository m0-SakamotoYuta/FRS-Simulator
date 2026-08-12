import os

import cv2
import numpy as np
import open3d as o3d
import pyrealsense2 as rs


# ============================================================
# ユーザー設定
# ============================================================

SAVE_DIRECTORY = r"C:/Users/bio/PycharmProjects/hayashi/python3.9/Tagawa code/a"  # PCD保存先
PCD_FILENAME_PREFIX = "pcd_"  # PCDファイル名の先頭文字

FRAME_RATE = 15  # フレームレート [fps]
DEPTH_WIDTH = 1280  # 深度画像の幅 [pixel]
DEPTH_HEIGHT = 720  # 深度画像の高さ [pixel]
COLOR_WIDTH = 1280  # カラー画像の幅 [pixel]
COLOR_HEIGHT = 720  # カラー画像の高さ [pixel]

DISCARD_FRAMES_BEFORE_RECORDING = 45  # 撮影開始前に破棄するフレーム数
PREVIEW_SCALE = 0.5  # プレビュー表示倍率
DEPTH_TRUNC_M = 0.60  # 点群生成時に使用する最大距離 [m]

MANUAL_DEPTH_EXPOSURE = False  # True: 手動露光、False: 自動露光
DEPTH_EXPOSURE = 5000  # 手動露光時の露光値
def configure_depth_sensor(depth_sensor):
    """深度センサーの露光を設定する。"""
    try:
        if MANUAL_DEPTH_EXPOSURE:
            if depth_sensor.supports(rs.option.enable_auto_exposure):
                depth_sensor.set_option(rs.option.enable_auto_exposure, 0)
                print("Depth Auto Exposure: OFF")

            if depth_sensor.supports(rs.option.exposure):
                depth_sensor.set_option(rs.option.exposure, DEPTH_EXPOSURE)
                print(f"Depth Exposure set to: {DEPTH_EXPOSURE}")
            else:
                print("警告: 手動露光はサポートされていません。")
        elif depth_sensor.supports(rs.option.enable_auto_exposure):
            depth_sensor.set_option(rs.option.enable_auto_exposure, 1)
            print("Depth Auto Exposure: ON")
    except Exception as error:
        print(f"警告: 露光設定中にエラーが発生しました: {error}")

def discard_frames(pipeline):
    """撮影開始前のフレームを破棄する。"""
    print("\n--- 撮影準備開始 ---")
    print(
        f"最初の {DISCARD_FRAMES_BEFORE_RECORDING} "
        "フレームを破棄します。"
    )

    for discard_index in range(DISCARD_FRAMES_BEFORE_RECORDING):
        remaining_frames = (
            DISCARD_FRAMES_BEFORE_RECORDING - discard_index
        )
        remaining_seconds = remaining_frames / FRAME_RATE
        print(
            f"\r撮影開始まで: {remaining_seconds:.1f} 秒 "
            f"({remaining_frames:02d}フレーム)",
            end="",
        )
        pipeline.wait_for_frames()

    print("\r撮影開始まで: 0.0 秒 (00フレーム)")


def draw_center_distance(image, depth_frame):
    """画像中央のマーカーと距離を描画する。"""
    height, width = image.shape[:2]
    center_x = width // 2
    center_y = height // 2
    distance = depth_frame.get_distance(center_x, center_y)

    cv2.line(
        image,
        (center_x - 10, center_y),
        (center_x + 10, center_y),
        (255, 255, 255),
        1,
    )
    cv2.line(
        image,
        (center_x, center_y - 10),
        (center_x, center_y + 10),
        (255, 255, 255),
        1,
    )
    cv2.putText(
        image,
        f"{distance:.3f} m",
        (center_x + 15, center_y - 15),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )


def record_frames(pipeline, align):
    """
    スペースキーで撮影を開始・停止し、生のFramesetを保持する。

    長時間撮影すると、保持したFramesetの分だけメモリ使用量が増える。
    """
    colorizer = rs.colorizer()
    recorded_framesets = []
    is_recording = False

    print("\nプレビューを表示します。")
    print("スペースキー: 撮影開始／撮影停止")
    print("ESCキー: 終了")

    while True:
        success, frames = pipeline.try_wait_for_frames(timeout_ms=100)

        if not success:
            if cv2.waitKey(1) & 0xFF == 27:
                break
            continue

        # 録画中は処理負荷を抑えるため生フレームを使用する。
        # プレビュー中だけdepthをcolor座標系へalignする。
        if is_recording:
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
        else:
            aligned_frames = align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        color_rgb = np.asanyarray(color_frame.get_data())
        color_bgr = cv2.cvtColor(color_rgb, cv2.COLOR_RGB2BGR)
        draw_center_distance(color_bgr, depth_frame)

        if is_recording:
            cv2.putText(
                color_bgr,
                f"REC  Frames: {len(recorded_framesets)}",
                (10, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.circle(color_bgr, (25, 60), 8, (0, 0, 255), -1)

        cv2.imshow(
            "color",
            cv2.resize(
                color_bgr,
                None,
                fx=PREVIEW_SCALE,
                fy=PREVIEW_SCALE,
            ),
        )

        if not is_recording:
            colored_depth = colorizer.colorize(depth_frame)
            depth_bgr = cv2.cvtColor(
                np.asanyarray(colored_depth.get_data()),
                cv2.COLOR_RGB2BGR,
            )
            draw_center_distance(depth_bgr, depth_frame)
            cv2.imshow(
                "depth",
                cv2.resize(
                    depth_bgr,
                    None,
                    fx=PREVIEW_SCALE,
                    fy=PREVIEW_SCALE,
                ),
            )

        key = cv2.waitKey(1) & 0xFF

        if key == 27:
            print("\nESCキーが押されました。")
            break

        if key == ord(" "):
            if not is_recording:
                discard_frames(pipeline)
                try:
                    cv2.destroyWindow("depth")
                except cv2.error:
                    pass

                is_recording = True
                recorded_framesets = []
                print("\n--- 連続撮影開始 ---")
                # 破棄前に取得したframesを保存しない。
                continue

            print("\n--- 連続撮影停止 ---")
            is_recording = False
            break

        if is_recording:
            # カメラ停止後にも使用できるようSDKのフレーム参照を保持する。
            frames.keep()
            recorded_framesets.append(frames)

            if len(recorded_framesets) % FRAME_RATE == 0:
                seconds = len(recorded_framesets) / FRAME_RATE
                print(
                    f"\r撮影中: {len(recorded_framesets)}フレーム "
                    f"(約{seconds:.1f}秒)",
                    end="",
                )

    return recorded_framesets


def save_point_clouds(
    recorded_framesets,
    align,
    camera_intrinsic,
    depth_scale,
    save_directory,
):
    """保持したFramesetをalignし、連番PCDとして保存する。"""
    if not recorded_framesets:
        print("撮影データがないため、PCDは保存しません。")
        return

    print(
        f"\n{len(recorded_framesets)}フレームのPCD生成を開始します。"
    )

    saved_count = 0

    for index, stored_frames in enumerate(recorded_framesets, start=1):
        index_text = f"{index:06d}"

        try:
            aligned_frames = align.process(stored_frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()

            if not color_frame or not depth_frame:
                print(
                    f"PCD生成スキップ: align後のフレーム取得失敗 "
                    f"({index_text})"
                )
                continue

            color_data = np.array(
                color_frame.get_data(),
                dtype=np.uint8,
                copy=True,
            )
            depth_data = np.array(
                depth_frame.get_data(),
                dtype=np.uint16,
                copy=True,
            )

            color_image = o3d.geometry.Image(color_data)
            depth_meters = depth_data.astype(np.float32) * depth_scale
            depth_image = o3d.geometry.Image(depth_meters)

            rgbd_image = (
                o3d.geometry.RGBDImage.create_from_color_and_depth(
                    color_image,
                    depth_image,
                    depth_scale=1.0,
                    depth_trunc=DEPTH_TRUNC_M,
                    convert_rgb_to_intensity=False,
                )
            )

            point_cloud = (
                o3d.geometry.PointCloud.create_from_rgbd_image(
                    rgbd_image,
                    camera_intrinsic,
                )
            )

            if not point_cloud.has_points():
                print(
                    f"PCD生成スキップ: 点がありません ({index_text})"
                )
                continue

            point_cloud.transform(
                [
                    [1, 0, 0, 0],
                    [0, -1, 0, 0],
                    [0, 0, -1, 0],
                    [0, 0, 0, 1],
                ]
            )

            save_path = os.path.join(
                save_directory,
                f"{PCD_FILENAME_PREFIX}{index_text}.pcd",
            )

            if o3d.io.write_point_cloud(save_path, point_cloud):
                saved_count += 1
            else:
                print(f"PCD保存失敗: {save_path}")

            if index % FRAME_RATE == 0:
                progress = index * 100.0 / len(recorded_framesets)
                print(
                    f"PCD生成中: {index}/{len(recorded_framesets)} "
                    f"({progress:.1f}%)"
                )
        except Exception as error:
            print(
                f"PCD生成エラー ({index_text}): {error}"
            )

    print(
        f"PCD保存完了: {saved_count}/"
        f"{len(recorded_framesets)}フレーム"
    )
    print(f"保存先: {save_directory}")


def main():
    save_directory = os.path.normpath(SAVE_DIRECTORY)
    os.makedirs(save_directory, exist_ok=True)

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(
        rs.stream.depth,
        DEPTH_WIDTH,
        DEPTH_HEIGHT,
        rs.format.z16,
        FRAME_RATE,
    )
    config.enable_stream(
        rs.stream.color,
        COLOR_WIDTH,
        COLOR_HEIGHT,
        rs.format.rgb8,
        FRAME_RATE,
    )
    align = rs.align(rs.stream.color)

    try:
        profile = pipeline.start(config)
    except RuntimeError as error:
        print(f"RealSense D405を起動できませんでした: {error}")
        return

    recorded_framesets = []

    try:
        print("RealSense D405を起動しました。")

        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        print(f"Depth Scale: {depth_scale}")
        configure_depth_sensor(depth_sensor)

        color_profile = profile.get_stream(
            rs.stream.color
        ).as_video_stream_profile()
        intrinsics = color_profile.get_intrinsics()
        camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
            intrinsics.width,
            intrinsics.height,
            intrinsics.fx,
            intrinsics.fy,
            intrinsics.ppx,
            intrinsics.ppy,
        )

        recorded_framesets = record_frames(pipeline, align)
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        print("\nRealSense D405を停止しました。")

    save_point_clouds(
        recorded_framesets,
        align,
        camera_intrinsic,
        depth_scale,
        save_directory,
    )


if __name__ == "__main__":
    main()
