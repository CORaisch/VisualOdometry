#ifndef _LOGGING_HPP
#define _LOGGING_HPP

#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>

static std::ofstream logfile;

class Logging
{
public:
    /* constructors */
    Logging()
    {
        b_log_to_cmdline = false;
        b_log_to_file = false;
        logfilename = "null";
    }

    Logging(bool _f_cmdline, const bool _f_logging, const std::string& filepath)
    {
        #ifdef NDEBUG
            b_log_to_cmdline = false;
        #else
            b_log_to_cmdline = true;
        #endif

        // if user activated logging from cmdline, use logging independent from build mode
        b_log_to_cmdline = _f_cmdline;

        // set if logfile should be written
        b_log_to_file = _f_logging;

        // open logfile
        double cpuTime = double(cv::getTickCount())/cv::getTickFrequency();
        logfilename = filepath + "log_" + std::to_string(cpuTime) + ".txt";

        // notify user about existance of logfile
        std::cout << "logs will be written to \"" << logfilename << "\" if logging is enabled" << std::endl;
    }

    /* operators */
    // only use this version for unformatted text, i.e. text without cmd coloring
    void operator()(const std::string& _msg)
    {
        // write to cmd line
        writeToCMD(_msg);
        // always write to logfile
        writeToLog(_msg);
    }

    // use this version for formatted text, i.e. text with bash style cmd coloring
    void operator()(const std::string& _crBegin, const std::string& _formatterBegin, const std::string& _msg, const std::string& _formatterEnd, const std::string& _crEnd)
    {
        // write formatted to cmd line
        writeToCMD(_crBegin + _formatterBegin + _msg + _formatterEnd + _crEnd);
        // write unformatted to logfile
        writeToLog(_crBegin + _msg + _crEnd);
    }

private:
    /* private member variablse */
    bool b_log_to_cmdline, b_log_to_file;
    std::string logfilename;

    /* private member methodes */
    void writeToCMD(std::string const& _msg)
    {
        // write log to commandline if flag is set
        if(b_log_to_cmdline)
            std::cout << _msg << std::flush;
    }

    void writeToLog(std::string const& _msg)
    {
        // write message to log file
        if(b_log_to_file)
        {
            logfile.open(logfilename, std::ios::app);
            logfile << _msg;
            logfile.close();
        }
    }

};

#endif //_LOGGING_HPP
