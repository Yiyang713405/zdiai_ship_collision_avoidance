import requests
import json
import time

URL = 'http://118.89.198.174:10083'
USER = 'ccs1001'


class HttpAPI:

    def __init__(self, url, user):
        self.url = url
        self.user = user
        self.headers = {
            'Content-Type': 'application/json'
        }
        self.S = requests.session()

    def get_osmodel(self):
        data = {
            'openid': self.user
        }
        url = self.url + '/get_osmodel'
        headers = self.headers
        r = self.S.get(url, data=json.dumps(data), headers=headers)
        return json.loads(r.text)

    def get_osdata(self):
        data = {
            'openid': self.user
        }
        url = self.url + '/get_osdata'
        headers = self.headers
        r = self.S.get(url, data=json.dumps(data), headers=headers)
        return json.loads(r.text)

    def get_tsdata(self):
        data = {
            'openid': self.user
        }
        url = self.url + '/get_tsdata'
        headers = self.headers
        r = self.S.get(url, data=json.dumps(data), headers=headers)
        return json.loads(r.text)

    def get_envdata(self):
        data = {
            'openid': self.user
        }
        url = self.url + '/get_envdata'
        headers = self.headers
        r = self.S.get(url, data=json.dumps(data), headers=headers)
        return json.loads(r.text)

    def get_scenedata(self):
        data = {
            'openid': self.user
        }
        url = self.url + '/get_scenedata'
        headers = self.headers
        r = self.S.get(url, data=json.dumps(data), headers=headers)
        return json.loads(r.text)

    def post_ctrl_os_general(self, steering_mode, ap_course_order, fu_rudder_order,
                             tele_port, tele_stbd, thrus_stbd, thrus_port):
        data = {
            'openid': self.user,
            "steering_mode": steering_mode,
            "autopilot": {
                "course_order": ap_course_order
            },
            "telegraph": {
                "order1": tele_port,
                "order2": tele_stbd
            },
            "rudder": {
                "order": fu_rudder_order
            },
            "thruster": {
                "order1": thrus_port,
                "order2": thrus_stbd
            }
        }
        url = self.url + '/ctrl_os_general'
        headers = self.headers
        r = self.S.post(url, data=json.dumps(data), headers=headers)
        return json.loads(r.text)

    def post_ctrl_os_azm(self, steering_mode, ap_course_order,
                         azm_thrus_port, azm_angle_port,
                         azm_thrus_stbd, azm_angle_stbd,
                         thrus_port, thrus_stbd):
        data = {
            "openid": self.user,
            "steering_mode": steering_mode,
            "autopilot": {
                "course_order": ap_course_order
            },
            "azimuthal": {
                "thrust_order1": azm_thrus_port,
                "angle_order1": azm_angle_port,
                "thrust_order2": azm_thrus_stbd,
                "angle_order2": azm_angle_stbd
            },
            "thruster": {
                "order1": thrus_port,
                "order2": thrus_stbd
            }
        }
        url = self.url + '/ctrl_os_azm'
        headers = self.headers
        r = self.S.post(url, data=json.dumps(data), headers=headers)
        return json.loads(r.text)

if __name__ == "__main__":
    api = HttpAPI(URL, USER)
    OS_data = api.get_osdata()
    print(OS_data)
    print(OS_data["data"]["lon"])
    # print(api.get_osmodel())
    # print(api.get_osdata())
    # print(api.get_tsdata())
    # print(api.get_envdata())
    # print(api.get_scenedata())
    # print(api.post_ctrl_os_general('fu', 0, 0, 0, 0, 0, 0))
    # print(api.post_ctrl_os_azm('fu', 0, 10, 0, 10, 0, 0, 0))
