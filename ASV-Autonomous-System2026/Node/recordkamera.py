import cv2
import datetime
import os

def record_ship_pov():
    # 1. Inisialisasi Kamera (0 adalah ID kamera bawaan/USB)
    cap = cv2.VideoCapture('/dev/video0')

    if not cap.isOpened():
        print("[ERROR] Gagal membuka kamera! Pastikan kamera sudah terhubung.")
        return
    # 2. Ambil Properti Kamera (Resolusi)
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = 30.0  # Kecepatan frame per detik

    # 3. Buat Folder Penyimpanan jika Belum Ada
    output_dir = "recordings"
    os.makedirs(output_dir, exist_ok=True)

    # 4. Penamaan File Berdasarkan Waktu / Timestamp
    ship_prefix = "trial lomba"
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    filename = f"{ship_prefix}_POV_{timestamp}.mp4"
    filepath = os.path.join(output_dir, filename)

    # 5. Inisialisasi VideoWriter
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(filepath, fourcc, fps, (frame_width, frame_height))

    print("=" * 50)
    print(f" MEREKAM POV KAPAL: {ship_prefix}")
    print(f" File disimpan ke  : {filepath}")
    print(f" Resolusi Video    : {frame_width}x{frame_height} @ {fps} FPS")
    print(" Tekan [Ctrl + C] untuk menghentikan rekaman.")
    print("=" * 50 + "\n")

    # 6. Loop Perekaman Headless (Tanpa GUI/imshow)
    frame_count = 0
    try:
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                print("\n[WARNING] Gagal menerima frame dari kamera. Menghentikan...")
                break

            # Tulis frame ke file mp4
            out.write(frame)
            frame_count += 1

            # Tampilkan indikator status di terminal setiap 30 frame (~1 detik)
            if frame_count % 30 == 0:
                seconds = int(frame_count / fps)
                print(f"\r[MEREKAM] Durasi: {seconds} detik ({frame_count} frames)", end="", flush=True)

    except KeyboardInterrupt:
        print("\n\n[INFO] Sinyal penghentian diterima (Ctrl+C).")

    finally:
        # 7. Bersihkan Resource Kamera dan File
        cap.release()
        out.release()
        print("\n[SELESAI] File rekaman telah berhasil disimpan dengan aman.")

if __name__ == "__main__":
    record_ship_pov()