param(
    [string]$pyScript
)

$env:PATH+=";C:\Users\Ahmed Amine Chafik\Softs\SOFA\build\bin\Release\"
$env:PYTHONPATH+=";C:\Users\Ahmed Amine Chafik\Softs\SOFA\plugins\STLIB\python3\src"
$env:PYTHONPATH+=";C:\Users\Ahmed Amine Chafik\Softs\SOFA\projects\EmioLabs\assets"
$env:PATH+=";C:\Users\Ahmed Amine Chafik\AppData\Local\Programs\Python\Python310\"
runSofa.exe -l SofaPython3,SofaImGui -g imgui $pyScript