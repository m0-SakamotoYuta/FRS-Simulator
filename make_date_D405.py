import pyrealsense2 as rs
import numpy as np
import cv2
import open3d as o3d
import os
from datetime import datetime


# --- ユーザー設定項目 ---
SAVE_DIRECTORY = r"C:/Users/bio/PycharmProjects/hayashi/python3.9/Tagawa code/PCD/test"

FRAME_RATE = 15

# D405は近距離用なので、必要に応じて変更してください
# 例：0.3m以内だけ使うなら 0.3、60cmまで使うなら 0.6
DEPTH_TRUNC = 0.6


def create_point_cloud_from_frames(color_frame, depth_frame, depth_scale, pinhole_camera_intrinsic):
    """
    RealSenseのcolor_frameとdepth_frameからOpen3Dの点群を作成する関数
    """

    color_image_np = np.asanyarray(color_frame.get_data())
    depth_image_np = np.asanyarray(depth_frame.get_data())

    # Open3D用画像へ変換
    color_o3d = o3d.geometry.Image(color_image_np)

    # 深度画像をメートル単位へ変換
    depth_m_np = depth_image_np.astype(np.float32) * depth_scale
    depth_o3d = o3d.geometry.Image(depth_m_np)

    # RGBD画像を作成
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_o3d,
        depth_o3d,
        depth_scale=1.0,
        depth_trunc=DEPTH_TRUNC,
        convert_rgb_to_intensity=False
    )

    # RGBD画像から点群を作成
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd_image,
        pinhole_camera_intrinsic
    )

    if not pcd.has_points():
        return None

    # Open3Dで見やすい座標系に変換
    pcd.transform([
        [1,  0,  0, 0],
        [0, -1,  0, 0],
        [0,  0, -1, 0],
        [0,  0,  0, 1]
    ])

    return pcd


def main():
    # 保存先ディレクトリ作成
    save_directory = os.path.normpath(SAVE_DIRECTORY.replace("\\", "/"))

    if not os.path.exists(save_directory):
        os.makedirs(save_directory)
        print(f"保存ディレクトリを作成しました: {save_directory}")

    # --- RealSense D405 初期化 ---
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, FRAME_RATE)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, FRAME_RATE)

    pipeline = rs.pipeline()
    profile = None

    try:
        profile = pipeline.start(config)
        print("RealSense D405を起動しました。")

        # 深度スケール取得
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        print(f"Depth Scale: {depth_scale}")

        # D405用：元コードと同様にalignは使わず、そのまま取得する
        # カメラ内部パラメータ取得
        color_stream_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()

        if color_stream_profile is None:
            print("エラー: RealSenseカメラのカラープロファイルの取得に失敗しました。")
            return

        intr = color_stream_profile.get_intrinsics()

        pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
            intr.width,
            intr.height,
            intr.fx,
            intr.fy,
            intr.ppx,
            intr.ppy
        )

        print("カメラ内部パラメータを取得しました。")
        print(f"width: {intr.width}, height: {intr.height}")
        print(f"fx: {intr.fx}, fy: {intr.fy}, ppx: {intr.ppx}, ppy: {intr.ppy}")

        # 深度画像を色付けするためのcolorizer
        colorizer = rs.colorizer()

        print("\nプレビューウィンドウが表示されます。")
        print("スペースキーを押すと、その瞬間の点群を作成・可視化・保存します。")
        print("ESCキーを押すと、撮影せずに終了します。")

        captured = False

        while True:
            success, frames = pipeline.try_wait_for_frames(timeout_ms=100)

            if not success:
                key_check = cv2.waitKey(1) & 0xFF
                if key_check == 27:
                    print("ESCキーが押されたため終了します。")
                    break
                continue

            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if not color_frame or not depth_frame:
                continue

            # --- RGB画像の表示 ---
            color_image_raw = np.asanyarray(color_frame.get_data())
            display_color_image = cv2.cvtColor(color_image_raw, cv2.COLOR_RGB2BGR)

            # --- 深度画像の表示 ---
            colored_depth_frame = colorizer.colorize(depth_frame)
            depth_colormap_bgr = np.asanyarray(colored_depth_frame.get_data())
            depth_colormap_bgr = cv2.cvtColor(depth_colormap_bgr, cv2.COLOR_RGB2BGR)

            # 中心点の距離を取得
            h, w, _ = depth_colormap_bgr.shape
            cy, cx = h // 2, w // 2
            center_distance = depth_frame.get_distance(cx, cy)

            # 深度画像の中心に十字マークを描画
            cv2.line(depth_colormap_bgr, (cx - 10, cy), (cx + 10, cy), (255, 255, 255), 1)
            cv2.line(depth_colormap_bgr, (cx, cy - 10), (cx, cy + 10), (255, 255, 255), 1)

            # 深度画像に中心距離を表示
            cv2.putText(
                depth_colormap_bgr,
                f"{center_distance:.3f} m",
                (cx + 15, cy - 15),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
                cv2.LINE_AA
            )

            # RGB画像にも操作説明を表示
            cv2.putText(
                display_color_image,
                "Press SPACE to capture / ESC to exit",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
                cv2.LINE_AA
            )

            # プレビュー表示
            cv2.imshow("color", display_color_image)
            cv2.imshow("depth", depth_colormap_bgr)

            key = cv2.waitKey(1) & 0xFF

            if key == ord(" "):
                print("\nスペースキーが押されました。")
                print("現在のフレームから点群を作成します...")

                pcd = create_point_cloud_from_frames(
                    color_frame=color_frame,
                    depth_frame=depth_frame,
                    depth_scale=depth_scale,
                    pinhole_camera_intrinsic=pinhole_camera_intrinsic
                )

                if pcd is None:
                    print("点群が生成されませんでした。")
                    print("DEPTH_TRUNCが小さすぎる、または深度が正しく取得できていない可能性があります。")
                    continue

                print(f"点群数: {len(pcd.points)}")

                # 撮影後はOpenCVのプレビューを閉じる
                cv2.destroyAllWindows()

                # 点群を可視化
                print("点群を可視化します。ウィンドウを閉じるとPCDとして保存します。")
                o3d.visualization.draw_geometries(
                    [pcd],
                    window_name="D405 Captured Point Cloud",
                    width=1280,
                    height=720
                )

                # PCD保存
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                pcd_filename = f"d405_single_shot_{timestamp}.pcd"
                save_path = os.path.join(save_directory, pcd_filename)

                success_save = o3d.io.write_point_cloud(save_path, pcd)

                if success_save:
                    print(f"PCDファイルを保存しました: {save_path}")
                else:
                    print("PCDファイルの保存に失敗しました。")

                captured = True
                break

            elif key == 27:
                print("ESCキーが押されたため終了します。")
                break

        if not captured:
            print("撮影は行われませんでした。")

    except RuntimeError as e:
        print(f"RealSenseカメラのエラー: {e}")

    finally:
        if profile is not None:
            pipeline.stop()
            print("RealSenseカメラを停止しました。")

        cv2.destroyAllWindows()
        print("処理が完了しました。")


if __name__ == "__main__":
    main()