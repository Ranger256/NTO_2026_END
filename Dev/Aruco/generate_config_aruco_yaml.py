count_x = 5
count_y = 7

size = 0.3

step_x = 0.9
step_y = 0.9

id_marker = 0

output_file = 'model.config'

with open(output_file, "w", encoding="utf-8") as f:
    for x in range(count_x):
        for y in range(count_y):
            f.write(f'{id_marker} 0.3 {x*step_x} {y*step_y} 0.0\n')

            #f.write(f"- id: {id_marker}\n")
            #f.write(f"  size: {size}\n")
            #f.write(f"  position: [{x*step_x}, {y*step_y}, 0]\n")

            id_marker += 1
