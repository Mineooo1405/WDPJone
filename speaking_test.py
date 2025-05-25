from gtts import gTTS
import os
import re

def get_next_filename(base_path="./audio", prefix="speech_", extension=".mp3"):
    """Tìm số thứ tự tiếp theo cho file audio"""
    
    # Đảm bảo thư mục tồn tại
    os.makedirs(base_path, exist_ok=True)
    
    # Lấy danh sách các file hiện có
    files = os.listdir(base_path)
    
    # Lọc các file khớp với mẫu prefix + số + extension
    max_index = 0
    pattern = re.compile(f"^{prefix}(\\d+){extension}$")
    
    for file in files:
        match = pattern.match(file)
        if match:
            try:
                index = int(match.group(1))
                max_index = max(max_index, index)
            except ValueError:
                continue
    
    # Tăng số thứ tự lên 1
    next_index = max_index + 1
    
    # Tạo tên file mới theo định dạng
    next_filename = os.path.join(base_path, f"{prefix}{next_index}{extension}")
    
    return next_filename

text = """
One object that I always keep with me is my old leather wallet. It's not brand new or stylish, but it means a lot to me because it was a gift from my dad when I started university.

He gave it to me on the day I moved into my dorm. I still remember him handing it over and saying, "This will carry more than just money — it should carry your responsibility too." That really stuck with me. Since then, I've kept it with me every single day.

It's a bit worn out now, but it still holds everything I need — my student ID, a photo of my family, a few lucky coins, and even that little note from my dad. It holds a lot of memories and reminds me of where I come from and what I'm working toward.

Even though I could easily buy a new one, I choose not to. This wallet is like a little piece of home that I carry with me. Every time I take it out, it gives me a sense of comfort and motivation.

To sum up, it's not about how it looks or how much it costs — it's about the meaning behind it. Like people say, sometimes the smallest things take up the most room in your heart — and that's exactly how I feel about this wallet.
"""

# Ví dụ với gTTS (Google Text-to-Speech)
tts = gTTS(text=text, lang='en', slow=False)

# Lấy tên file mới với số thứ tự tăng dần
output_file = get_next_filename()

# Lưu file với tên mới
tts.save(output_file)
print(f"Đã lưu file âm thanh: {output_file}")

# Phát file (Windows) - sử dụng tên file đã tạo
# Sử dụng path tuyệt đối để đảm bảo phát đúng file
abs_path = os.path.abspath(output_file)
os.system(f'start "" "{abs_path}"')  # 'start' giúp mở ứng dụng mặc định trên Windows