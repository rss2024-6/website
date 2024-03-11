import os
lab_name = "lab3"
def generate_scroll_container():
    html = '<div class="scroll-container">\n'

    folder = lab_name
    files = os.listdir(folder)

    for file in files:
        if file.endswith('.mp4'):
            html += f'  <video id = "{file.replace(".mp4", "")}" style="max-height: 300px; border-radius: 15px;" controls><source src="{folder}/{file}" type="video/mp4">Your browser does not support the video tag.</video>\n'

    html += '</div>\n'

    return html

if __name__ == "__main__":
    generated_html = generate_scroll_container()

    with open(f'{lab_name}.html', 'w') as file:
        file.write(generated_html)