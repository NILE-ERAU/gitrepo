import subprocess

python_bin = "~/.virtualenvs/py3cv4/bin/activate"
script_file = "save_image.py"

subprocess.Popen([python_bin, script_file])
