"""PC blue-pair classification from calibrated chroma directions, not lightness.

Scores and margins are diagnostic distances, not calibrated probabilities.
Ambiguous pixels can propose a body but never vote for a color identity.
"""
import numpy as np


def pc_calibrated_threshold(color,samples,material):
    """PC-specific black bounds measured against the saved black/cyan images."""
    threshold,state=material.calibrated_threshold(color,samples)
    threshold=list(threshold)
    if color[0]==5:  # UI/calibration5 = black; its wire ID is6.
        threshold[1]=min(threshold[1],32)
        threshold[2]=max(threshold[2],-12);threshold[3]=min(threshold[3],12)
        threshold[4]=max(threshold[4],-12);threshold[5]=min(threshold[5],12)
    return threshold,state


def black_pixel_gate(lab,hsv):
    """Black is dark and near-neutral, not merely the dark side of another color."""
    chroma=np.linalg.norm(lab[:,:,1:],axis=2)
    colored=(hsv[:,:,1]>=55)&(hsv[:,:,2]>=20)&(chroma>=5)
    return (lab[:,:,0]<=32)&(chroma<=12)&~colored


class BluePairModel:
    def __init__(self, samples):
        self.centers = {}
        self.state = 'NEED_PAIR'
        self.separation = 0.
        for key in (3, 6):
            values = np.asarray(samples.get(str(key), []), np.float32)
            if values.size == 0:
                continue
            chroma = values[:, 1:]
            lengths = np.linalg.norm(chroma, axis=1)
            useful = lengths >= 10
            if not np.any(useful):
                continue
            center = np.median(chroma[useful] / lengths[useful, None], axis=0)
            self.centers[key] = center / max(float(np.linalg.norm(center)), 1e-6)
        if len(self.centers) == 2:
            self.separation = float(np.linalg.norm(self.centers[3] - self.centers[6]))
            self.state = 'CHROMA_OK' if self.separation >= .22 else 'CHROMA_OVERLAP'

    def classify(self, lab, hsv):
        chroma = lab[:, :, 1:]
        lengths = np.linalg.norm(chroma, axis=2)
        direction = chroma / np.maximum(lengths[:, :, None], 1e-6)
        # Blue/cyan proposal excludes neutral shadows, paper, green and purple.
        family = ((lengths >= 10) & (lab[:, :, 2] < -6) &
                  (hsv[:, :, 1] >= 45) & (hsv[:, :, 0] >= 80) & (hsv[:, :, 0] <= 142))
        labels = np.zeros(lengths.shape, np.uint8)
        margin = np.zeros(lengths.shape, np.float32)
        if self.state != 'CHROMA_OK':
            return labels, family, margin
        distances = np.stack([np.linalg.norm(direction - self.centers[key], axis=2) for key in (3, 6)])
        nearest = np.min(distances, axis=0)
        family &= nearest <= .50
        margin = np.abs(distances[0] - distances[1])
        reliable = family & (margin >= max(.10, self.separation * .18))
        labels[reliable & (distances[0] < distances[1])] = 3
        labels[reliable & (distances[1] < distances[0])] = 6
        return labels, family, margin

    def describe_region(self, mask, labels, margins):
        selected = mask != 0
        size = max(1, int(np.count_nonzero(selected)))
        fractions = {key: float(np.count_nonzero((labels == key) & selected) / size) for key in (3, 6)}
        color = max(fractions, key=fractions.get)
        coverage = fractions[color]
        difference = coverage - fractions[9 - color]
        reliable = self.state == 'CHROMA_OK' and coverage >= .65 and difference >= .25
        return {'color': color if reliable else 0, 'coverage': coverage, 'vote_margin': difference,
                'chroma_margin': float(np.median(margins[selected])) if np.any(selected) else 0.,
                'state': 'COLOR_OK' if reliable else self.state if self.state != 'CHROMA_OK' else 'COLOR_AMBIGUOUS'}
