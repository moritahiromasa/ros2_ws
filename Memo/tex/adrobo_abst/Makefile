b-thesis_abst.pdf: b-thesis_abst.dvi
	dvipdfmx -f font.map tmp.dvi
	mv tmp.pdf b-thesis_abst.pdf

b-thesis_abst.dvi: *.tex
	sed -e 's/。/．/g' -e 's/、/，/g' b-thesis_abst.tex > tmp.tex
	platex tmp.tex
	#pbibtex tmp.aux
	platex tmp.tex
	platex tmp.tex

clean:
	rm -f *.aux *.log *.dvi *.bbl *.blg *.pdf *.ilg *.idx *.toc *.ind
