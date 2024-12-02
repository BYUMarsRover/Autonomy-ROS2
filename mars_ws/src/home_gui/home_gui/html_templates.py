CONNECTED_STR = "✔️"
# CONNECTED_STR = "Con"
DISCONNECTED_STR = "✖️"
# DISCONNECTED_STR = "Dis"
RED_HEX_STR = "a40000"
GREEN_HEX_STR = "4e9a06"
DEV_LINE = '<p style=" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;">\
                <span style=" color:#{status_color};">{status_str}</span>\
                <span style=" color:#000000;">{name}</span>\
            </p>'

def generate_status_html(dev_name, is_connected):
    if is_connected:
        status_color = GREEN_HEX_STR
        status_str = CONNECTED_STR
    else:
        status_color = RED_HEX_STR
        status_str = DISCONNECTED_STR
    return DEV_LINE.format(name=dev_name, status_color=status_color, status_str=status_str)
