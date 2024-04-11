import mistune
import re
import os

class LaTeXRenderer(mistune.HTMLRenderer):
    def block_math(self, text):
        # Handle block LaTeX. You might want to replace this with an actual LaTeX to HTML conversion
        return f'<div class="math">{text}</div>'

    def inline_math(self, text):
        # Handle inline LaTeX
        return f'<span class="math">{text}</span>'
class LaTeXMarkdown(mistune.Markdown):
    def __init__(self, renderer):
        super().__init__(renderer=renderer)
        # Add block and inline rules
        self.block.rules.append('math')
        self.inline.rules.append('math')

        # Define regex for block and inline math
        self.block.register('math', re.compile(r'^\$\$(.*?)\$\$', re.S), self.parse_math)
        self.inline.register('math', re.compile(r'\$(.*?)\$', re.S), self.parse_inline_math)

    def parse_math(self, m, state):
        return 'block_math', m.group(1)

    def parse_inline_math(self, m, state):
        return 'inline_math', m.group(1)
def make_markedown(directory, filename):
    readme = os.path.join(directory, filename)
    if not os.path.isfile(readme):
        return

    with open(os.path.join(directory, "html.txt"), 'w+') as f:
        with open(readme, 'r') as readme_f:
            renderer = LaTeXRenderer()
            markdown = LaTeXMarkdown(renderer=renderer)
            data = readme_f.read()
            f.write(markdown(data))
make_markedown(os.getcwd(), "labs/5/report.md")