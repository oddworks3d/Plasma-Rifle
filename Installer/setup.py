"""
This is an example of py2app setup.py script for freezing your pywebview
application
Usage:
    python setup.py py2app
"""
import os
from setuptools import setup


ENTRY_POINT = ['installer.py']

OPTIONS = {'argv_emulation': False,
           'strip': True,
           'iconfile': 'icon.icns', # uncomment to include an icon
           'includes': ['WebKit', 'Foundation', 'webview','serial','sys','glob','requests']}

setup(
    app=ENTRY_POINT,
    data_files=['render.html','m.uf2','pyboard.py','picnic.css','style.css'],
    options={'py2app': OPTIONS},
    setup_requires=['py2app'],
)