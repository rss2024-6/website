# Adding a report

Write up the report in markdown then sse the md_make.py to turn it into html. Copy this html into a file called `report.html` in the same folder as the weeks lab. You should see the markdown get rendered on the page

# The website

See the deployed website at [https://rss2024-6.github.io/website/](https://rss2024-6.github.io/website/)

# Embedding slides

## TODO

## convert md to html

pandoc --toc --standalone --mathml -f markdown -t html labs/5/report.md -o labs/5/report.html
