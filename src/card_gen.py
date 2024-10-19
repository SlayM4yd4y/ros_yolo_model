from PIL import Image, ImageDraw, ImageFont
import os

class CardGenerator:
    def __init__(self, output_dir, num_images):
        self.output_dir = output_dir
        self.num_images = num_images

    def generate_card(self, card_type):
        img = Image.new('RGB', (640, 400), color = (73, 109, 137))
        d = ImageDraw.Draw(img)
        font = ImageFont.load_default()
        
        text = "Hallgatói kártya" if card_type == "student" else "Alkalmazotti kártya"
        d.text((10,10), text, font=font, fill=(255,255,0))
        
        img_name = f"{card_type}_card.png"
        img_path = os.path.join(self.output_dir, img_name)
        img.save(img_path)
        return img_path

    def generate_cards(self):
        os.makedirs(self.output_dir, exist_ok=True)
        for i in range(self.num_images):
            card_type = "student" if i % 2 == 0 else "employee"
            self.generate_card(card_type)

if __name__ == "__main__":
    output_dir = "generated_cards"
    num_images = 100
    generator = CardGenerator(output_dir, num_images)
    generator.generate_cards()
