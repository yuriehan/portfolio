# Analysis of COVID-19 Effects in NYC

**Tech:** R, RMarkdown, Shiny, SQL, Data Visualization  

This project investigates the effects of COVID-19 on New York City and the general population using statistical modeling, data analysis, and web scraping. The analysis focuses on demographic correlations and trends in highly populated areas.

### Features
- Leveraged **R** (with `dplyr`, `ggplot2`, `Shiny`) and **SQL** to transform, clean, and analyze COVID-19 datasets.
- Created **12 visualizations** including stacked area charts and correlation analyses to uncover relationships between demographic factors and COVID-19 effects.
- Developed an **R Markdown file** (`analysis.Rmd`) that combines code, analysis, and narrative in a single reproducible document.

### Files
- `analysis.Rmd` → Main analysis file; can be knit into HTML, PDF, or Word.
- `final-report.pdf` → Compiled project report.
- `presentation.pdf` → Slides summarizing methods, findings, and insights.
- `data/` → (Optional) Contains datasets used for analysis.

### How to Run
1. Open `analysis.Rmd` in **RStudio**.  
2. Ensure required packages are installed:  
   ```r
   install.packages(c("dplyr", "ggplot2", "Shiny"))
