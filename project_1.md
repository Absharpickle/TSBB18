**<a href="https://gitlab.liu.se/cvl/tsbb18/-/blob/master/README.md">&larr; Tillbaka till huvudsidan</a>**

# Delprojekt 1: Legodetektion

I delprojekt 1 ska varje grupp sätta ihop ett system för live lego-detektion med hjälp av en kamera och en Raspberry Pi. Systemet ska kunna

- Analysera strömmande video från kameran.
- Detektera och rita centrum för röda, gröna och blåa legobitar i videoströmmen.
- Köra i realtid på Raspberry Pi.

Ett ytterligare krav är att ni själva måste kompilera OpenCV för Raspberry Pi.

## Tips för delprojekt 1
 1. Bortsett från OpenCV får ni installera diverse program eller bibliotek med <code>sudo apt-get install ...</code>
 2. Hämta **<a href="https://github.com/opencv/opencv">källkoden</a>** för OpenCV (använd *inte* <code>git clone</code>, för ni behöver inte historiken, och den tar STOR plats).
 3. Leta upp instruktioner för hur man kompilerar OpenCV, och försök följa.
 Om ni får ett felmeddelande som säger att någonting saknas (g++, make, cmake, ???), installera det och försök kompilera på nytt.
 4. Om kompileringen lyckas (den kan ta lång tid!) behöver ni göra så att Python hittar cv2-modulen.
 Beroende på hur ni gör finns det olika lösningar här: något steg ni kör kan till exempel innehålla "install", eller så kan ni köra <code>export PYTHONPATH=<sökväg till katalogen som innehåller cv2-modulen></code>.
