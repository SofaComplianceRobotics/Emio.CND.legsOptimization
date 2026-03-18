param(
    [string]$pyScript
)

$SOFA_ROOT = "C:\Users\Ahmed Amine Chafik\Softs\SOFA"

$env:PATH += ";$SOFA_ROOT\build\bin\Release\"
$env:PYTHONPATH += ";$SOFA_ROOT\plugins\STLIB\python3\src"
$env:PYTHONPATH += ";$SOFA_ROOT\projects\EmioLabs\assets"
$env:PATH += ";C:\Users\Ahmed Amine Chafik\AppData\Local\Programs\Python\Python310\"

runSofa.exe -l SofaPython3,SofaImGui -g imgui $pyScript