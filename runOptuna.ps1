param([int]$trials,[string]$pyScript)

$env:SOFA_ROOT = "C:\Users\Ahmed Amine Chafik\Softs\SOFA\build"
$env:PATH += ";$env:SOFA_ROOT\bin\"
$env:PYTHONPATH += ";$env:SOFA_ROOT\lib\python3\site-packages\"
$env:PYTHONPATH += ";C:\Users\Ahmed Amine Chafik\Softs\SOFA\projects\EmioLabs\assets"
$env:PYTHONPATH +=";C:\Users\Ahmed Amine Chafik\Softs\SOFA\plugins\STLIB\python3\src"


python ./optimize.py -n $trials $pyScript
