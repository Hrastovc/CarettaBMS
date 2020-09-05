:: Batch file to generate a reference manual from the comments in the source code, with
:: Doxygen. The following tools should be installed and added to the path:
:: - DoxyGen (doxygen.exe)
:: - Graphviz (dot.exe)
:: - MikTex (pdflatex.exe)
if exist ..\..\Doc\CarettaBMS_ReferenceManual.pdf del ..\..\Doc\CarettaBMS_ReferenceManual.pdf
doxygen.exe Doxyfile
call .\output\latex\make.bat
call copy .\output\latex\refman.pdf ..\..\Doc\CarettaBMS_ReferenceManual.pdf
::pause