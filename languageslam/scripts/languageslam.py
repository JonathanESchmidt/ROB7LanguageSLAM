#!/usr/bin/env python

import sys

from my_module import MyPlugin
from rqt_gui.main import Main

plugin = 'languageslam'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))

