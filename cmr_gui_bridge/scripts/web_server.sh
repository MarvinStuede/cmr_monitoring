#!/bin/bash
p=$(rospack find cmr_gui_bridge)
cd $p/gui
python -m SimpleHTTPServer
