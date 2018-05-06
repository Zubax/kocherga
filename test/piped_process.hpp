/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 Zubax Robotics
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 */

#pragma once

#include <memory>
#include <optional>
#include <csignal>
#include <unistd.h>
#include <sys/prctl.h>
#include <fcntl.h>


namespace piped_process
{
/**
 * Represents an external OS process which has its stdin/stdout connected to the local process.
 */
class PipedProcess final
{
    friend std::shared_ptr<PipedProcess> launch(const std::string& command);

    const ::pid_t child_pid_;
    const int input_fd_;
    const int output_fd_;

    PipedProcess(::pid_t pid, int input, int output) :
        child_pid_(pid),
        input_fd_(input),
        output_fd_(output)
    { }

public:
    PipedProcess(const PipedProcess&) = delete;
    PipedProcess& operator=(const PipedProcess&) = delete;

    ~PipedProcess()
    {
        (void) ::kill(child_pid_, SIGTERM);
        (void) ::close(input_fd_);
        (void) ::close(output_fd_);
        (void) ::kill(child_pid_, SIGKILL);
    }

    ::pid_t getPID() const { return child_pid_; }

    int getInputFD()  const { return input_fd_; }
    int getOutputFD() const { return output_fd_; }

    /**
     * Reads the stdout of the child. Returns an empty option on failure.
     */
    std::optional<std::size_t> readOutput(void* buffer, std::size_t size)
    {
        const auto res = ::read(output_fd_, buffer, size);
        return (res < 0) ? std::optional<std::size_t>{} : std::optional<std::size_t>(std::size_t(res));
    }

    /**
     * Write the stdin of the child. Returns an empty option on failure.
     */
    std::optional<std::size_t> writeInput(const void* buffer, std::size_t size)
    {
        const auto res = ::write(input_fd_, buffer, size);
        return (res < 0) ? std::optional<std::size_t>{} : std::optional<std::size_t>(std::size_t(res));
    }

    /**
     * Makes the pipes non-blocking for use with IO multiplexing API and timeouts.
     */
    void makeIONonBlocking()
    {
        {
            const int flags = ::fcntl(input_fd_, F_GETFL, 0);
            (void) ::fcntl(input_fd_, F_SETFL, unsigned(flags) | unsigned(O_NONBLOCK));
        }
        {
            const int flags = ::fcntl(output_fd_, F_GETFL, 0);
            (void) ::fcntl(output_fd_, F_SETFL, unsigned(flags) | unsigned(O_NONBLOCK));
        }
    }
};

using PipedProcessPtr = std::shared_ptr<PipedProcess>;

/**
 * Instantiates PipedProcess by creating a new process.
 * Returns a null pointer in case of failure.
 */
inline PipedProcessPtr launch(const std::string& command)
{
    int in_fd[2]{};
    int out_fd[2]{};

    if (::pipe(in_fd) != 0)
    {
        return {};
    }

    if (::pipe(out_fd) != 0)
    {
        return {};
    }

    const ::pid_t pid = ::fork();

    if (pid < 0)
    {
        return {};
    }

    if (pid == 0)       // We're in the forked process, must not return
    {
        (void) ::close(out_fd[0]);
        (void) ::close(in_fd[1]);

        if (::dup2(in_fd[0], STDIN_FILENO) < 0)
        {
            ::exit(1002);
        }

        if (::dup2(out_fd[1], STDOUT_FILENO) < 0)
        {
            ::exit(1001);
        }

        (void) ::prctl(PR_SET_PDEATHSIG, SIGKILL);       // Kill the child if the parent dies

        // Mimicking system().
        (void) ::execl("/bin/sh", "sh", "-c", command.c_str(), static_cast<char*>(nullptr));
        ::exit(1000);
    }

    (void) ::close(out_fd[1]);
    (void) ::close(in_fd[0]);

    return PipedProcessPtr(new PipedProcess(pid, in_fd[1], out_fd[0]));
}

}
