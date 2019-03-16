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
        bLog_to_cmdline = false;
        logfilename = "null";
    }

    Logging(bool _f_logging, const std::string& filepath)
    {
        #ifdef NDEBUG
            bLog_to_cmdline = false;
        #else
            bLog_to_cmdline = true;
        #endif
        // if user activated logging from cmd line, use logging independent from build mode
        bLog_to_cmdline = _f_logging;

        // open logfile
        double cpuTime = double(cv::getTickCount())/cv::getTickFrequency();
        logfilename = filepath + "log_" + std::to_string(cpuTime) + ".txt";

        // notify user about existance of logfile
        std::cout << "logs will be written to \"" << logfilename << "\"" << std::endl;
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
    bool bLog_to_cmdline;
    std::string logfilename;

    /* private member methodes */
    void writeToCMD(std::string const& _msg)
    {
        // write log to commandline if flag is set
        if(bLog_to_cmdline)
            std::cout << _msg << std::flush;
    }

    void writeToLog(std::string const& _msg)
    {
        // write message to log file
        logfile.open(logfilename, std::ios::app);
        logfile << _msg;
        logfile.close();
    }

};

#endif //_LOGGING_HPP