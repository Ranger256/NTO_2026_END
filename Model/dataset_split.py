#!/usr/bin/env python3
import shutil
import random
from pathlib import Path

def split_dataset():
    # Пути
    source_dir = Path("/home/ilya/collected_data")
    output_dir = Path("/home/ilya/tree_person_dataset")
    
    # Создайте структуру папок
    for split in ['train', 'val', 'test']:
        (output_dir / "images" / split).mkdir(parents=True, exist_ok=True)
        (output_dir / "labels" / split).mkdir(parents=True, exist_ok=True)
    
    # Получите все изображения
    images = list(source_dir.glob("*.jpg"))
    random.seed(42)
    random.shuffle(images)
    
    n = len(images)
    train_imgs = images[:int(n * 0.7)]   # 70%
    val_imgs = images[int(n * 0.7):int(n * 0.9)]  # 20%
    test_imgs = images[int(n * 0.9):]    # 10%
    
    print(f"\n📊 Dataset split:")
    print(f"   Train: {len(train_imgs)} images")
    print(f"   Val:   {len(val_imgs)} images")
    print(f"   Test:  {len(test_imgs)} images")
    print(f"   Total: {n} images\n")
    
    # Копирование файлов
    for split_name, split_imgs in [('train', train_imgs), ('val', val_imgs), ('test', test_imgs)]:
        for img_path in split_imgs:
            # Копируем изображение
            shutil.copy(img_path, output_dir / "images" / split_name / img_path.name)
            
            # Копируем аннотацию
            lbl_path = img_path.with_suffix('.txt')
            if lbl_path.exists():
                shutil.copy(lbl_path, output_dir / "labels" / split_name / lbl_path.name)
    
    print("✅ Dataset split complete!")
    print(f"📁 Saved to: {output_dir}\n")

if __name__ == '__main__':
    split_dataset()