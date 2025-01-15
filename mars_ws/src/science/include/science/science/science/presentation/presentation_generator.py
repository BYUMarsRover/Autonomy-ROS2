from odpslides.presentation import Presentation
import os
from matplotlib import pyplot as plt

# num sites parameter. We will iterate this many times, starting at 1, going until numSites
numSites = 10

LIFE_THRESHOLD = 100
NO_LIFE_THRESHOLD = 50
DIRT_THRESHOLD = 20


DIR = os.path.join(os.path.dirname(__file__), "resources")

filenames = sorted(os.listdir(DIR))

# initialize the Presentation object
P = Presentation()

# title slide for the presentaiton
P.add_title_chart(title="URC Science Report", subtitle="BYU Mars Rover Team")

for filename in filenames:
    site = f"Site {int(filename[0]) + 1} "
    if "plot" in filename:
        if filename[-4:] != '.txt':
            continue
        title = site + "Biuret photoresistor test"

        # Get values
        with open(os.path.join(DIR, filename), 'r') as f:
            values = f.read().split()
            # y_axis = range(0, 500, 10)
        values = [int(v) for v in values]

        # Plot values
        plt.ylim(0, max(max(values) + 10, 120))
        plt.plot(values)

        # Labels
        plt.ylabel('Photoresistor Values')
        plt.xlabel('Time')
        # plt.legend()
        plt.legend(['Photoresistor']) # , 'No Life', 'Life', 'Dirt'])

        # Save, clear, add to presentation
        plt.savefig(os.path.join(DIR, filename[:-4] + '.png'), format='png')
        plt.clf()
        P.add_titled_image(image_file=os.path.join(DIR, filename[:-4] + '.png'), title=title)
    elif ".txt" == filename[-4:]:
        title = site + "Notes"
        with open(os.path.join(DIR, filename), "r") as f:
            text = f.read()
            P.add_titled_outline_chart(title=title, outline=text)
    else:
        title = site + "Image"
        P.add_titled_image(image_file=os.path.join(DIR, filename), title=title)

# save the presentation and then launch (launch=1). If you just want to save, then set launch=0
P.save(filename="presentation.odp", launch=1)