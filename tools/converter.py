from PIL import Image
import paho.mqtt.publish as publish
import time

blacklist = [1, 12, 13, 15, 24, 28, 31, 32, 33, 34, 42, 43, 45, 52]

with Image.open('chars.png') as img:
  pixels = img.convert("RGB").load()

with open('../embedded/images.h', 'w', newline='\n') as outfile:
  outfile.write('// DO NOT EDIT THIS FILE! WILL BE OVERWRITTEN DURING BUILD!\n')
  outfile.write('// Images CC0 by https://egordorichev.itch.io/chare\n')
  outfile.write('#include <Arduino.h>\n\n')
  outfile.write('const byte IMAGES[][64*2] PROGMEM = {\n')

  image_count = 0
  for imgNo in range(54):
    if imgNo in blacklist:
      print("Skipping image " + str(imgNo))
      continue

    img = []
    imgX = imgNo % 9
    imgY = int(imgNo/9)
    print(imgX, imgY)
    offsetX = 8*(imgX+1)+4*(imgX)
    offsetY = 8*(imgY+1)+4*(imgY)
    for i in range(offsetY, offsetY+8):
      for j in range(offsetX, offsetX+8):
        (r, g, b) = pixels[j, i]
        c = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)
        # Make background black
        if c == 0x0823:
          img.append(0x00)
          img.append(0x00)
        else:
          img.append(c >> 8)
          img.append(c & 0xFF)

    matrix_data = bytearray(img)
    #publish.single("nitek/draw", bytearray(img), hostname="broker.emqx.io")

    if image_count == 0:
      c_code = '\t{\n'
    else:
      c_code = '\t,{\n'
    data_length = len(matrix_data)
    for i in range(len(matrix_data)):
      if (i % 12) == 0:
        c_code += '\t\t'
      c_code += '0x%02x' % matrix_data[i]

      if (i + 1) < data_length:
        c_code += ','
        if (i % 12) == 11:
          c_code += '\n'
        else:
          c_code += ' '
    c_code += '\n\t}\n'

    outfile.write(c_code)
    image_count += 1

  outfile.write('\n};\n\n')
  outfile.write('const size_t IMAGE_COUNT = ' + str(image_count) + ';')

