from pandocfilters import toJSONFilter


def strip_links(key, value, fmt, _):
    # strip readme badges
    if key == 'Para':
        if value[0]['t'] == 'Link' and value[0]['c'][0][0]['t'] == 'Image':
            return []

    # turn links to text
    if key == 'Link':
        return value[0]

    # strip raw html, logo, and ruled lines
    if key in ['RawBlock', 'HorizontalRule', 'Image']:
        return []


if __name__ == "__main__":
    toJSONFilter(strip_links)

    # debugging
    # with open('README.json') as f:
    #     doc = json.load(f)
    #     walk(doc, strip_links, '', '')
