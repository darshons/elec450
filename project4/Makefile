# Source directory for LaTeX files
SRC_DIR = src

# Output directory for PDF and intermediary products
PDF_DIR = pdf

# Fill this out with which files to compile in SRC_DIR
DOCUMENTS = project4

.PHONY: all clean $(DOCUMENTS)
all : $(DOCUMENTS)

clean:
	$(RM) $(PDF_DIR)/*

# Compile LaTeX files
TEX = pdflatex -shell-escape -interaction=nonstopmode -file-line-error -output-directory $(PDF_DIR)
$(DOCUMENTS): % : $(SRC_DIR)/%.tex
	mkdir -p $(PDF_DIR)
	$(TEX) $<
	$(TEX) $<
