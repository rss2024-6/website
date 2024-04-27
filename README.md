# Adding a report

Write up the report in markdown then sse the md_make.py to turn it into html. Copy this html into a file called `report.html` in the same folder as the weeks lab. You should see the markdown get rendered on the page

# The website

See the deployed website at [https://rss2024-6.github.io/website/](https://rss2024-6.github.io/website/)

# Embedding slides

## TODO

## convert md to html

pandoc --toc --standalone --mathml -f markdown -t html labs/5/report.md -o labs/5/report.html

# Adding gifs to report

format your links like this
you can find the file id from the url link for example in
`https://drive.google.com/file/d/10Fsay2f2ZI2YQg0jf0R5W792qAn87o1F/view` the file id is `10Fsay2f2ZI2YQg0jf0R5W792qAn87o1F`

`https://drive.google.com/thumbnail?id=[FILE ID]&sz=w1000`
