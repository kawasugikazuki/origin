# -*- coding:utf-8 -*-
import cv2
import numpy as np
import os
import time

# --- Local ---
from FrameDivision import FrameDivision

class CentralRecognition() :

    __OUTPUT_DIR = 'output/'
    __MARK_DIR = 'mark/'
    __IMG_EXT = '.png'

    # Sampling points
    __N = 32

    # FPS
    __dt = 1 / 30

    # Window Function
    __win = np.hanning(__N).astype(np.float32)

    # Frequency Axis
    __freq = np.linspace(0, 1.0 / __dt, __N)
    '''
    [ 0.          0.96774194  1.93548387  2.90322581  3.87096774  4.83870968
    5.80645161  6.77419355  7.74193548  8.70967742  9.67741935 10.64516129
    11.61290323 12.58064516 13.5483871  14.51612903 15.48387097 16.4516129
    17.41935484 18.38709677 19.35483871 20.32258065 21.29032258 22.25806452
    23.22580645 24.19354839 25.16129032 26.12903226 27.09677419 28.06451613
    29.03225806 30.        ]
    '''

    # Frequency Infimum (__freq[2] = 1.93[Hz])
    __freq_infimum = 2

    # Number of markers on one pole
    __num_marker = 2

    # Flag Default(False) or BackgroundSubtractorCNT(True)
    __flag_backgroundSubtractor = True

    # Output BackgroundSubtractorCNT Video
    __flag_video = False

    ## This is the constructor of this class
    # @brief Creates a output directory
    # @param None
    # @return None
    def __init__(self) :
        self.__frame_divider = FrameDivision()
        self.__fgbg = cv2.bgsegm.createBackgroundSubtractorCNT(minPixelStability=0, maxPixelStability = 30 * 1)
        if not os.path.exists(self.__OUTPUT_DIR) :
            os.makedirs(self.__OUTPUT_DIR)

        if not os.path.exists(self.__MARK_DIR) :
            os.makedirs(self.__MARK_DIR)

    def __getFrames(self, video_name) :
        frames = self.__frame_divider.Frame_Division(video_name)
        self.__img_size = frames.shape[1:]
        return frames

    def __backgroundSubtractorCNT(self, frames):
        self.__fgmask = np.array([self.__fgbg.apply(_) for _ in frames])
        if self.__flag_video :
            self.__output_backgroundSubtractorVideo()
        self.__valid_region = (np.sum(self.__fgmask, axis=0) * 255) / self.__fgmask.max()
        self.__valid_region = self.__valid_region.astype(bool)
        valid_region_frames = frames[:, self.__valid_region]
        return valid_region_frames

    def __output_backgroundSubtractorVideo(self) :
        # Set Video Writer
        fourcc = cv2.VideoWriter_fourcc('m','p','4', 'v')
        video  = cv2.VideoWriter('video.mp4', fourcc, 30.0, tuple(np.roll(self.__img_size, 1)), 0)

        for i in range(self.__N):
            video.write(self.__fgmask[i])

        video.release()

    def __window_func(self, frames) :
        # Default
        if not self.__flag_backgroundSubtractor :
            frames = frames.transpose(1, 2, 0)
            frames = frames * np.hanning(self.__N)
            frames = frames.transpose(2, 0, 1)

        # BackgroundSubtractorCNT
        else :
            frames = frames.T * self.__win

        acf = 1/(self.__win.sum()/self.__N)

        return frames, acf

    def __fft(self, frames) :
        # Default
        if not self.__flag_backgroundSubtractor :
             fft_signal = np.fft.fft(frames, axis=0)

        # BackgroundSubtractorCNT
        else :
            frames = frames.transpose(1, 0)
            fft_signal = np.fft.fft(frames, axis=0)

        return fft_signal

    def __normalize(self, fft_signal, acf) :
        amp = acf * np.abs(fft_signal)
        amp = amp / (self.__N / 2)

        # Default
        if not self.__flag_backgroundSubtractor :
            amp[0, :, :] /= 2

        # BackgroundSubtractorCNT
        else :
            amp[0, :] /= 2

        return amp

    def __bpf(self, freq, amp) :
        # Default
        if not self.__flag_backgroundSubtractor :
            amp_max = np.max(amp[self.__freq_infimum:int(self.__N/2)], axis=0)
            amp_argmax = np.argmax(amp[self.__freq_infimum:int(self.__N/2), :, :], axis=0)

        # BackgroundSubtractorCNT
        else :
            amp_max = np.max(amp[self.__freq_infimum:int(self.__N/2)], axis=0)
            amp_argmax = np.argmax(amp[self.__freq_infimum:int(self.__N/2), :], axis=0)

        filtered_amp_max = amp_max[self.__freq_infimum + amp_argmax == freq]

        return amp_max, amp_argmax, filtered_amp_max

    def __binarization(self, freq, amp_max, amp_argmax, filtered_amp_max, output) :
        filtered_amp_max[filtered_amp_max.max() > filtered_amp_max] = 0

        if filtered_amp_max.max() < 10 :
            filtered_amp_max[filtered_amp_max.max() <= filtered_amp_max] = 0

        else :
            filtered_amp_max[filtered_amp_max.max() <= filtered_amp_max] = 255

        if not self.__flag_backgroundSubtractor :
            output[self.__freq_infimum + amp_argmax == freq] = filtered_amp_max

        else :
            amp_max[self.__freq_infimum + amp_argmax == freq] = filtered_amp_max
            amp_max[self.__freq_infimum + amp_argmax != freq] = 0
            output[self.__valid_region] = amp_max

        return output

    def __output_result(self, freq, output, video_name) :
        cv2.imwrite(os.path.join(self.__OUTPUT_DIR, video_name + '-' + str(freq) + self.__IMG_EXT), output)
        result = output.reshape(output.shape[:2])
        result = result.astype(bool)
        index = np.array(np.where(result == True))
        marker_center = np.sum(index,axis=1)/ index.shape[1]
        marker_center = np.roll(marker_center,1)
        mark = cv2.cvtColor(output,cv2.COLOR_GRAY2BGR)
        cv2.circle(mark,center=(tuple(marker_center.astype(np.int))),radius=50,color=(0,255,0),thickness=5,lineType=cv2.LINE_4,shift=0)
        tmp = cv2.imread(os.path.join(self.__MARK_DIR,'tmp' + self.__IMG_EXT))
        mark = cv2.addWeighted(mark,0.5,tmp,0.5,0)
        cv2.imwrite(os.path.join(self.__MARK_DIR, video_name + '-' + str(freq) + self.__IMG_EXT),mark)

    def __each_detect_marker_pole(self, amp, freq, video_name) :
        output = np.zeros((self.__img_size), dtype=np.uint8)
        for freq in range(freq, freq+2, 1) :
            print(freq)
            amp_max, amp_argmax, filtered_amp_max = self.__bpf(freq, amp)
            output = self.__binarization(freq, amp_max, amp_argmax, filtered_amp_max, output)
            self.__output_result(freq, output, video_name)
            self.__result = np.vstack((self.__result, output.reshape(tuple(np.insert(self.__img_size, 0, 1)))))

    def __detect_marker_pole(self, amp, freq, video_name) :
        self.__result = np.empty(shape=(tuple(np.insert(self.__img_size, 0, 0))), dtype=np.uint8)
        for freq in freq :
            self.__each_detect_marker_pole(amp, freq, video_name)

    def main(self, video_name : str, freq : list) :
        frames = self.__getFrames(video_name)
        if self.__flag_backgroundSubtractor :
            frames = self.__backgroundSubtractorCNT(frames)
        frames, acf = self.__window_func(frames)
        fft_signal = self.__fft(frames)
        amp = self.__normalize(fft_signal, acf)
        self.__detect_marker_pole(amp, freq, video_name)
        return self.__result

    def __del__(self):
        del self.__frame_divider

if __name__=="__main__":
    start_time = time.time()
    centralRecognizer = CentralRecognition()
    VIDEO_NAME = input('Enter The Video file name (The Video Exists In The Video Folder.)')
    centralRecognizer.central_recognition(VIDEO_NAME, 'Green')
    print(time.time() - start_time)