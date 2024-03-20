#!/usr/bin/env python3
import os
import mistune

def make_markedown(directory, filename):
    readme = os.path.join(directory, filename)
    if not os.path.isfile(readme):
        return

    # Open an output file
    with open(os.path.join(directory, "html.txt"), 'w+') as f:

        # Add the text from the README
        with open(readme, 'r') as readme_f:
            renderer = mistune.HTMLRenderer(escape=False)
            markdown = mistune.Markdown(renderer=renderer)
            data = readme_f.read()
            f.write(markdown(data))
        
if __name__ == "__main__":
    make_markedown(os.getcwd(), "labs/4/report.md")
