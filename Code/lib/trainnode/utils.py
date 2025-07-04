import json

class config_manager:
    def __init__(self):
        return
    
    def load_config(self, filename):
        self.filename = filename
        with open(self.filename, 'r') as json_file:
            self.config = json.load(json_file)

        self.devices = self.config["devices"]
        self.settings = self.config["settings"]
        return
    
    def save_config(self):
        with open(self.filename, 'w') as json_file:
            json_file.write(json.dumps(self.config))
        return 