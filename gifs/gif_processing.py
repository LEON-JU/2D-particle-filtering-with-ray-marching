from PIL import Image

def reverse_red_blue(image):
    # Convert image to RGB mode
    image_rgb = image.convert("RGB")
    # Create a blank image with the same size and mode
    new_image = Image.new("RGB", image_rgb.size)

    # Iterate through each pixel and reverse red and blue channels
    for x in range(image_rgb.width):
        for y in range(image_rgb.height):
            # Get RGB values of the current pixel
            r, g, b = image_rgb.getpixel((x, y))
            # Swap red and blue values
            new_image.putpixel((x, y), (b, g, r))

    return new_image

def reverse_gif(input_path, output_path, frame_delay):
    # Open the GIF file
    gif = Image.open(input_path)

    # Process each frame in the GIF
    frames = []
    durations = []
    for frame in range(gif.n_frames):
        gif.seek(frame)  # Go to the specified frame
        frame_image = gif.copy()  # Copy the current frame
        reversed_image = reverse_red_blue(frame_image)  # Reverse red and blue channels
        frames.append(reversed_image)  # Append the processed frame to the list
        # Get duration of the current frame
        durations.append(gif.info['duration'])

    # Save the processed frames as a new GIF file with frame delay
    frames[0].save(output_path, save_all=True, append_images=frames[1:], loop=0, duration=frame_delay, disposal=gif.info.get('disposal', 0))

# 输入和输出文件的路径
input_path = "output_robot.gif"
output_path = "particles.gif"
# 设置帧延迟时间（毫秒）
frame_delay = 20  # 100 毫秒

# 反向处理 GIF 文件并设置帧延迟时间
reverse_gif(input_path, output_path, frame_delay)
