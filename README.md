# Instruções para compilação

Para gerar o arquivo da biblioteca (libGeneralPID.dll) os seguintes comando devem ser executados. 

```powershell
gcc -c src\GeneralPID.c -o object\GeneralPID.o
gcc -shared -o lib\libGeneralPID.dll object\GeneralPID.o
```

O processo só é funcional para usuários do sistema operacional Windows. Para outros sistemas e outras arquiteturas o link a seguir deve ser consultado:

https://www.typhoon-hil.com/documentation/typhoon-hil-software-manual/References/advanced_c_function.html
