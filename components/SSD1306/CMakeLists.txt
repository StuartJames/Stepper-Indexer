set(srcs 	"src/SSD1306.c"
		 	"src/Fontinfo.c"
		 	"src/Softi2c.c"
		 	"src/Fonts/Calibri8pt.c"
			"src/Fonts/Calibri12pt.c"
			"src/Fonts/Calibri18pt.c"
			"src/Fonts/LCD_5x7.c"
			"src/Fonts/SansSerif8pt.c"
			"src/Fonts/SansSerif12pt.c"
			"src/Fonts/SevenSeg10pt.c"
			"src/Fonts/Tahoma8pt.c"
			"src/Fonts/Tahoma12pt.c"
			"src/Fonts/Verdana8pt.c"
			"src/Fonts/Verdana12pt.c"
			"src/Fonts/Verdana18pt.c")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS include
                    REQUIRES driver)
