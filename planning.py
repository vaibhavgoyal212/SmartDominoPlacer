import logging
import os
import csv
import numpy as np
from pprint import pprint


SIZE = (5, 5)
FONTS_PATH = './fonts/'
COLOR_CODES_PATH = './fonts/color_codes.csv'


class Planner():
    def __init__(self, size=SIZE, fonts_path=FONTS_PATH, color_codes_path=COLOR_CODES_PATH) -> None:
        self.font_size = size
        self.fonts_path = fonts_path
        self.fonts = self._read_fonts(self.fonts_path)
        self.color_codes = self._read_color_codes(color_codes_path)
        self.dominos = {}
        self.logger = logging.getLogger("Planner")

    def _read_fonts(self, path: str) -> dict:
        fonts = {}
        for file_name in os.listdir(path):
            alpha = file_name.split('.')[0]
            alpha_font = list(csv.reader(
                open(
                    os.path.join(path, file_name)
                )
            ))
            alpha_font = np.rot90(alpha_font, 3).tolist()
            fonts[alpha] = alpha_font
        return fonts
    
    def _read_color_codes(self, path: str) -> dict:
        color_codes = {}
        with open(path, 'r', encoding='utf8') as f:
            reader = csv.reader(f, delimiter=',')
            for row in reader:
                color_codes[row[0]] = row[1]
        return color_codes

    def convert_colors(self, colors: dict) -> dict:
        coded_colors = {}
        for (color, number) in colors.items():
            coded_colors[
                self.color_codes[color]
            ] = number
        return coded_colors

    def get_counts(self, alpha: str) -> dict:
        if alpha not in self.fonts.keys():
            return None
        font = self.fonts[alpha]
        counts = {}
        for row in font:
            for entry in row:
                if entry not in counts.keys():
                    counts[entry] = 1
                else:
                    counts[entry] += 1
        return counts
    
    def get_str_counts(self, text: str) -> dict:
        counts = {}
        for alpha in text: 
            alpha_counts = self.get_counts(alpha)
            for (color, number) in alpha_counts.items():
                if color not in counts.keys():
                    counts[color] = number
                else:
                    counts[color] += number
        return counts
    
    def get_all_counts(self) -> dict:
        counts = {}
        for alpha in self.fonts:
            alpha_counts = self.get_counts(alpha)
            counts[alpha] = alpha_counts
        return counts

    def load_dominos(self, dominos: dict) -> dict:
        for (color, number) in dominos.items():
            if color not in self.dominos.keys():
                self.dominos[color] = number
            else:
                self.dominos[color] += number
        return self.dominos
    
    def evaluate_count(self, text: str) -> bool:
        required_counts = self.get_str_counts(text)
        for (color, number) in required_counts.items():
            if color not in self.dominos.keys():
                self.logger.error(f"Color not in stock: {self.color_codes[color]}")
                return False
            if self.dominos[color] < number:
                self.logger.error(f"Not enough dominos for color: {self.color_codes[color]}; need {number}, have {self.dominos[color]}")
                return False
        return True
        
    def set_mission(self, text: str, scale=1.0) -> list:
        if scale != 1.0:
            raise NotImplementedError
        if not self.evaluate_count(text):
            return None
        return self.lay_out_task(text)

    def lay_out_task(self, text: str) -> list:
        domino_matrix = []
        for alpha in text:
            domino_matrix += self.fonts[alpha]
        return domino_matrix

    # TODO: preset APIs for laying a row of dominos


if __name__ == '__main__':
    planner = Planner()
    pprint(f"Color codes: {planner.color_codes}")
    dominos = {'0': 100, '1': 100}
    planner.load_dominos(dominos)
    # pprint(planner.convert_colors(planner.dominos))
    # pprint(f"S: {planner.get_counts('S')}")
    # pprint(f"D: {planner.get_counts('D')}")
    # pprint(f"P: {planner.get_counts('P')}")
    # pprint(planner.evaluate_count('SDP'))
    # pprint(f"SDPPPPPPP:  {planner.convert_colors(planner.get_str_counts('SDPPPPPPP'))}")
    # pprint(planner.evaluate_count('SDPPPPPPP'))
    # pprint(planner.get_all_counts())
    # pprint(fonts)
    # pprint(planner.fonts['S'])
    # pprint(planner.set_mission("HISP"))
    domino_matrix = planner.set_mission("SDP")
    truth_values = []
    for domino_row in domino_matrix:
        truth_row = [entry == '0' for entry in domino_row]
        truth_values.append(truth_row)
    pprint(truth_values)
        
