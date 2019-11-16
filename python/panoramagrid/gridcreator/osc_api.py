import json
from dataclasses import dataclass
from enum import Enum
from time import sleep
from urllib import request, parse

import cv2 as cv
import numpy as np


@dataclass
class OscApi:
    class PictureStates(Enum):
        IDLE = 1
        TAKE = 2
        STATUS = 3
        WAIT = 4

    base_url: str
    timeout: float
    picture_state: PictureStates = PictureStates.IDLE

    def request(self, url: str, data=None, headers=None, method: str = 'GET'):
        if headers is None:
            headers = {}

        return request.urlopen(request.Request(url, data, headers, method=method), timeout=self.timeout)

    def execute_command(self, command, parameters=None):
        if parameters is None:
            parameters = {}

        response = self.request(
            url=parse.urljoin(self.base_url, '/osc/commands/execute'),
            data=json.dumps({'name': command, 'parameters': parameters}).encode('ascii'),
            headers={'Content-Type': 'application/json'},
            method='POST',
        )

        return json.loads(response.read().decode('utf-8'))

    def command_status(self, id):
        response = self.request(
            url=parse.urljoin(self.base_url, '/osc/commands/status'),
            data=json.dumps({'id': id}).encode('ascii'),
            headers={'Content-Type': 'application/json'},
            method='POST',
        )

        return json.loads(response.read().decode('utf-8'))

    def take_picture(self):
        self.picture_state = self.PictureStates.TAKE
        response = self.execute_command('camera.takePicture')

        while response['state'] == 'inProgress':
            self.picture_state = self.PictureStates.STATUS
            response = self.command_status(response['id'])
            self.picture_state = self.PictureStates.WAIT
            sleep(1)

        if response['state'] != 'done':
            raise Exception
        
        return response['results']['fileUrl'].split('/')[-1]

    def live_preview(self):
        return self.request(
            url=parse.urljoin(self.base_url, '/osc/commands/execute'),
            data=json.dumps({'name': 'camera.getLivePreview'}).encode('ascii'),
            headers={'Content-Type': 'application/json'},
            method='POST',
        )

    @staticmethod
    def get_live_frame(file):
        correct_header = b'---osclivepreview---\r\nContent-type: image/jpeg\r\nContent-Length:'

        header = file.read(len(correct_header)).strip()

        if header != correct_header:
            raise ValueError(f"Bad header received: {header}")

        size = int(file.readline())
        file.readline()

        data = file.read(size)
        file.read(4)

        return cv.imdecode(np.frombuffer(data, np.uint8), cv.IMREAD_COLOR)
