from RPLCD.i2c import CharLCD




def main():
    lcd = CharLCD('PCF8574', 0x27)

    lcd.write_string("test")


if __name__ == "__main__":
    main()
