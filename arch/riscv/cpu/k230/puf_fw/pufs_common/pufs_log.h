/**
 * @file      pufs_log.h
 * @brief     PUFsecurity log interface
 * @copyright 2020 PUFsecurity
 */
/* THIS SOFTWARE IS SUPPLIED BY PUFSECURITY ''AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. TO THE FULLEST
 * EXTENT ALLOWED BY LAW, PUFSECURITY'S TOTAL LIABILITY ON ALL CLAIMS IN
 * ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES,
 * IF ANY, THAT YOU HAVE PAID DIRECTLY TO PUFSECURITY FOR THIS SOFTWARE.
 */

#ifndef __PUFS_LOG_H__
#define __PUFS_LOG_H__

#ifdef __cplusplus
extern "C" {
#endif

#define LOG_LEVEL_DEBUG   1
#define LOG_LEVEL_INFO    2
#define LOG_LEVEL_WARN    3
#define LOG_LEVEL_ERROR   4

#ifndef LOG_LEVEL
#define LOG_LEVEL LOG_LEVEL_WARN
#endif

#define PRINT(...) 

#define LOG_PRINT(level, str, ...) PRINT("[%s] %s(): " str "\n", level, __func__, ## __VA_ARGS__)

#if LOG_LEVEL <= LOG_LEVEL_DEBUG
    #define LOG_DEBUG(...) LOG_PRINT("DEBUG", __VA_ARGS__)
#else
    #define LOG_DEBUG(...) {}
#endif

#if LOG_LEVEL <= LOG_LEVEL_INFO
    #define LOG_INFO(...) LOG_PRINT("INFO", __VA_ARGS__)
#else
    #define LOG_INFO(...) {}
#endif

#if LOG_LEVEL <= LOG_LEVEL_WARN
    #define LOG_WARN(...) LOG_PRINT("WARN", __VA_ARGS__)
#else
    #define LOG_WARN(...) {}
#endif

#if LOG_LEVEL <= LOG_LEVEL_ERROR
    #define LOG_ERROR(...) LOG_PRINT("ERROR", __VA_ARGS__)
#else
    #define LOG_ERROR(...) {}
#endif

#ifdef __cplusplus
} // closing brace for extern "C"
#endif

#endif /*__PUFS_LOG_H__*/